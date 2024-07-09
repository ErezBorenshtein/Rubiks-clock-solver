import cv2
import numpy as np

from clock_buffer import BufferManager

buffer = BufferManager(10,9)
centers_buffer = BufferManager(15,9)

max_thresh = 255

top_min_threshold = 32
bottom_min_threshold = 54

kernel_rate = 1

circle_param1 = 300
circle_param2 = 20

threshold1 = 24
threshold2 = 46

def rad_to_deg(radians):
    return radians * (180 / np.pi)

def angle_to_hour(angle_deg):  
    num_of_hours = round(angle_deg / 30)
    hour = 3- num_of_hours
    if hour < 0:
        hour = 12+ hour
    return hour

def get_angle(hand, centre):
    x_h, y_h = hand
    x_c, y_c = centre

    x_diff = x_h - x_c
    y_diff = -(y_h - y_c)

    # Calculate angle using arctan2 to handle all quadrants
    angle = np.arctan2(y_diff, x_diff)
    
    # Convert angle to the range [0, 2π)
    if angle < 0:
        angle += 2 * np.pi
    deg = rad_to_deg (angle)
    return angle

def filter_contours_by_y_threshold (contours,y_threshold):
    filtered_contours = []
    for contour in contours:
        # Get the bounding rectangle of the contour
        x, y, w, h = cv2.boundingRect(contour)
        
        # Check if the contour is below the y_threshold
        if y_threshold == 0:
            filtered_contours.append(contour)
        if y_threshold > 0 and y > y_threshold:
            filtered_contours.append(contour)
        if y_threshold < 0 and y < -y_threshold:
            filtered_contours.append(contour)
    return filtered_contours

def get_circles_indices(circles):
    """
    Sorts the circles in an approximate 3x3 grid where coordinates vary.
    
    Parameters:
    circles (list of tuples): A list of circles, where each circle is represented by a tuple (x, y, radius).
    
    Returns:
    list of int: A list of indices sorted by the grid order.
    """
    # Convert the circles to a numpy array for easier manipulation
    circles = np.array(circles)
    
    # Sort circles by their y-coordinates
    sorted_by_y = circles[np.argsort(circles[:, 1])]
    
    # Create lists to store each row
    rows = [[], [], []]
    
    # Approximate a threshold to distinguish rows based on y-coordinates
    y_threshold = (sorted_by_y[-1, 1] - sorted_by_y[0, 1]) / 3
    
    # Assign circles to rows
    current_row = 0
    current_y = sorted_by_y[0, 1]
    for circle in sorted_by_y:
        if circle[1] > current_y + y_threshold:
            current_row += 1
            current_y = circle[1]
        rows[current_row].append(circle)
    
    # Sort each row by x-coordinates
    for i in range(3):
        rows[i] = sorted(rows[i], key=lambda c: c[0])
    
    # Flatten the rows and extract indices
    sorted_circles = [circle for row in rows for circle in row]
    sorted_indices = [np.where((circles == circle).all(axis=1))[0][0] for circle in sorted_circles]
    
    return sorted_indices

def furthest_point_within_circle(circles, sorted_indices,contours,offset):
    """
    Finds the furthest contour point within each circle.
    
    Parameters:
    circles (list of tuples): A list of circles, where each circle is represented by a tuple (x, y, radius).
    contours (list of numpy arrays): A list of contours from CV2.findContours.
    
    Returns:
    list of tuples: A list of points (x, y), one per circle, which are the furthest from the circle center.
    """
    furthest_points = []

    denoised_centers = centers_buffer.average_positions()

    for i in range(9):
        
        #cx, cy, radius = circle
        cx, cy, radius = denoised_centers[i][0],denoised_centers[i][1],circles[sorted_indices[i]][2],
        max_distance = -1
        furthest_point = None

        num_contours = 0
        for contour in contours:
            #if num_contours == 48:
            num_points = 0
            for point in contour:
                px, py = point[0]
                
                # Check if the point is within the circle
                if (px - cx) ** 2 + (py - cy) ** 2 <= (radius-offset) ** 2:
                    num_points +=1
                    distance = np.sqrt((px - cx) ** 2 + (py - cy) ** 2)
                    # Update the furthest point if this point is further
                    if distance > max_distance:
                        max_distance = distance
                        furthest_point = (px, py)
            num_contours +=1
        furthest_points.append(furthest_point)

    return furthest_points

def extract_image(img_grayscale, min_thresh, max_thresh, kernel_rate):
    _, thresh = cv2.threshold(img_grayscale, min_thresh, max_thresh, cv2.THRESH_BINARY)
    kernel = np.ones((kernel_rate, kernel_rate), np.uint8)
    thresh = cv2.dilate(thresh, kernel, iterations=1)
    edges = cv2.Canny(thresh, 100, 200)
    return thresh,edges

def extract_circle_image(img_grayscale, kernel_rate):
    thresh = apply_threshold(img_grayscale)
    kernel = np.ones((kernel_rate, kernel_rate), np.uint8)
    thresh = cv2.dilate(thresh, kernel, iterations=1)
    edges = cv2.Canny(thresh, 100, 200)
    return thresh,edges

def contains_none(arrays):
    return any(any(cell is None for cell in row) for row in arrays)

def read_clock(camera) -> list[int]:
    font = cv2.FONT_HERSHEY_SIMPLEX
    img2 = 0

    show_contour_num = 0

    while True:

        if camera is not None:
            _, img2 = camera.read()
        else:
            img2 = cv2.imread("images\\captured_image_0.jpg")
        
        if img2 is None:
            print("Error: Image not found.")
            return

        img_grayscale = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    
        img= img2.copy()
        # Thresholding part
        circle_thresh, circle_edges =   extract_circle_image(img_grayscale, kernel_rate)
        top_thresh, top_edges =         extract_image(img_grayscale, top_min_threshold,     max_thresh, kernel_rate)
        bottom_thresh, bottom_edges =   extract_image(img_grayscale, bottom_min_threshold,  max_thresh, kernel_rate)

        top_contours, hierarchy = cv2.findContours(top_edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        bottom_contours, hierarchy = cv2.findContours(bottom_edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        # Circle Detection Part
        circles = cv2.HoughCircles(circle_edges, cv2.HOUGH_GRADIENT, 1, 35, param1=circle_param1, param2=circle_param2, minRadius=32, maxRadius=55)
        if (circles is None or len(circles.shape) == 1):
            cv2.imshow('Final', img)
            cv2.waitKey(1)
            continue
        
        cv2.imshow('CIRCLE EDGES', circle_edges)
        cv2.imshow('CIRCLE THRESHOLDED IMAGE', circle_thresh)
        cv2.imshow('TOP THRESHOLDED IMAGE', top_thresh)
        cv2.imshow('BOTTOM THRESHOLDED IMAGE', bottom_thresh)

        circles = np.uint16(np.around(circles))
        circles = np.squeeze(circles)
        
        if (circles is None or len(circles.shape) == 1):
            continue
        if len(circles) != 9:
            for circle_num in range(len(circles)):
            
                maxrad = circles[circle_num][2]
                centre = (circles[circle_num, 0], circles[circle_num, 1])
                
                # Displaying Circle Around Clock
                cv2.circle(img, centre, maxrad, (20, 90, 230), 2)
                cv2.circle(img, centre, 2, (0, 0, 255), 3)
            show_final_img(img)

        if (len(circles) != 9):
            continue
        
        sorted_indices = get_circles_indices(circles)

        circles[sorted_indices[0], 2] =circles[sorted_indices[1], 2] =circles[sorted_indices[2], 2] = 33
        circles[sorted_indices[3], 2] =circles[sorted_indices[4], 2] =circles[sorted_indices[5], 2] = 33
        circles[sorted_indices[6], 2] =circles[sorted_indices[7], 2] =30
        circles[sorted_indices[8], 2] = 30

        centers_buffer.add_to_buffer([[circles[sorted_indices[0]][0],circles[sorted_indices[0]][1]],
                                      [circles[sorted_indices[1]][0],circles[sorted_indices[1]][1]],
                                      [circles[sorted_indices[2]][0],circles[sorted_indices[2]][1]],
                                      [circles[sorted_indices[3]][0],circles[sorted_indices[3]][1]],
                                      [circles[sorted_indices[4]][0],circles[sorted_indices[4]][1]],
                                      [circles[sorted_indices[5]][0],circles[sorted_indices[5]][1]],
                                      [circles[sorted_indices[6]][0],circles[sorted_indices[6]][1]],
                                      [circles[sorted_indices[7]][0],circles[sorted_indices[7]][1]],
                                      [circles[sorted_indices[8]][0],circles[sorted_indices[8]][1]]])

        #boundary between top and bottom circles
        y_threshold = (circles[sorted_indices[3], 1] + circles[sorted_indices[6], 1])/2


        # Getting and Displaying Contours
        top_contours, hierarchy = cv2.findContours(top_edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        top_contours = filter_contours_by_y_threshold(top_contours,-y_threshold)
        bottom_contours, hierarchy = cv2.findContours(bottom_edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        bottom_contours = filter_contours_by_y_threshold(bottom_contours,y_threshold)
        
        contours = top_contours + bottom_contours
        cv2.drawContours(img, contours, -1, (231, 48, 255), 1)
        furthest_points = furthest_point_within_circle(circles,sorted_indices,contours,0)
        

        #x, y, w, h = cv2.boundingRect(contours[show_contour_num])
        #cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        hours = []
        centers = []
        denoised_centers = centers_buffer.average_positions()
        for idx in range(len(furthest_points)):
            point = furthest_points[idx]
            if point is not None:
                circle = circles[sorted_indices[idx]]
                circle2 = denoised_centers[idx]
                cv2.circle(img, point, 5, (0, 255, 0), -1)  # Green dot
                angle_in_rad= get_angle(point, (circle2[0],circle2[1]))
                hour_angle = np.rad2deg(angle_in_rad)
                hour = angle_to_hour(hour_angle)
                hours.append(hour)
                cv2.putText(img,str(hour),point,font,1, (255, 255, 255), 2, cv2.LINE_AA)
        
        if hours != [] and len(hours) == 9:
            buffer.add_to_buffer(hours)


        cv2.putText(img,str(buffer.get_most_popular_numbers()),(0,20),font,1, (255, 255, 255), 2, cv2.LINE_AA)

        #for circle_num in range(len(sorted_indices)):
        for i in range(9):
            maxrad = circles[sorted_indices[i], 2]
            centre = (circles[i, 0], circles[i, 1])
            
            # Displaying Circle Around Clock
            cv2.circle(img, centre, maxrad, (20, 90, 230), 2)
            cv2.circle(img, centre, 2, (0, 0, 255), 3)
        
        if show_final_img(img):
            return hours
    

def show_final_img(img):
    global circle_param1
    global bottom_min_threshold
    global top_min_threshold
    global circle_param2
    global threshold1
    global threshold2
    

    cv2.imshow('Final', img)
    key = cv2.waitKey(1) & 0xFF
    
    if key == 27:  # ESC key to exit
        return False
    elif key == ord('1'):
        top_min_threshold -= 1
        print("top_min_threshold= ",top_min_threshold)
    elif key == ord('2'):
        top_min_threshold +=1
        print("top_min_threshold= ",top_min_threshold)
    elif key == ord('3'):
        threshold1 -=1
        print("threshold1= ",threshold1)
    elif key == ord('4'):
        threshold1 +=1 
        print("threshold1= ",threshold1)
    elif key == ord('5'):
        bottom_min_threshold -=1
        print("bottom_min_threshold= ",bottom_min_threshold)
    elif key == ord('6'):
        bottom_min_threshold +=1
        print("bottom_min_threshold= ",bottom_min_threshold)
    elif key == ord('7'):
        threshold2 -=1
        print("threshold2= ",threshold2)
    elif key == ord('8'):
        threshold2 +=1
        print("threshold2= ",threshold2)

    elif key == ord(' '):
        return True

def find_available_cameras(max_index=10):
    available_cameras = []
    for index in range(max_index):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            available_cameras.append(index)
            cap.release()
    return available_cameras

def apply_threshold(img):

    global threshold1
    global threshold2

    # Create a copy of the original image to store the result
    result = np.copy(img)

    y_split = (img.shape[0] // 3)*2

    # Apply the first threshold to the upper part of the image (y = 1 to y = 200)
    upper_part = img[:y_split, :]
    _, upper_thresh = cv2.threshold(upper_part, threshold1, 255, cv2.THRESH_BINARY)

    # Apply the second threshold to the lower part of the image (y = 200 to y = max)
    lower_part = img[y_split:, :]
    _, lower_thresh = cv2.threshold(lower_part, threshold2, 255, cv2.THRESH_BINARY)

    # Combine the two parts back into one image
    result[:y_split, :] = upper_thresh
    result[y_split:, :] = lower_thresh
    return result


def main():
    centers_buffer.prepare_positions(20,9)
    camera = cv2.VideoCapture(0)
    hour1 = read_clock(camera)
    hour2 = read_clock(camera)
    cv2.destroyAllWindows()
    
    print("hour1: ",hour1)
    print("hour2: ",hour2)
    



if __name__ == "__main__":
    main()
