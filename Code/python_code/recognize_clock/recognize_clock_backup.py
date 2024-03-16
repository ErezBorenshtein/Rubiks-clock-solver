import cv2
import numpy as np

font = cv2.FONT_HERSHEY_SIMPLEX

min_Radius=120 
max_Radius=200
param_1 = 270
param_2 = 35


def get_angle(hand,centre):
    angle=0
    #x_h=hand[0][0]
    #y_h=hand[0][1]
    x_h=hand[0]
    y_h=hand[1]
    x_c=centre[0]
    y_c=centre[1]

    x_diff=x_h-x_c
    y_diff=y_h-y_c
    x_diff=float(x_diff)
    y_diff=float(y_diff)

    if abs(x_diff)< 5:
        if y_diff>0:
            angle=np.deg2rad(180)
        else:
            angle=0
    elif(x_diff*y_diff>0):
        if(x_diff>=0 and y_diff>0):
            angle=np.pi-np.arctan(x_diff/y_diff)
        elif(x_diff<=0 and y_diff<0):
            angle=2*np.pi-np.arctan(x_diff/y_diff)
    elif(x_diff*y_diff<0):
        if(y_diff>=0 and x_diff<0):
            #angle=(3*np.pi)/4+np.arctan(x_diff/y_diff)
            angle=abs(np.arctan(x_diff/y_diff)) + np.pi
        elif(y_diff<=0 and x_diff>0):
            angle=-np.arctan(x_diff/y_diff)

    return angle

def get_hour(hourangle):

    fullhour=hourangle//30
    remainder=hourangle%30
    if(remainder>15):
        fullhour+=1
    if fullhour==0:
        return 12
    return int(fullhour)

def get_minsec(angle):
    minsec=angle/(np.rad2deg(2*np.pi))*60
    return int(minsec)

def get_farthest_point(contour, reference_point):
    # Initialize variables
    max_distance = -1
    farthest_point = None

    # Iterate through each contour
    for point in contour:
        # Calculate distance between reference point and current point
        distance = np.linalg.norm(point[0] - reference_point)
        
        # Update farthest point if current distance is greater
        if distance > max_distance:
            max_distance = distance
            farthest_point = tuple(point[0])
            
    return farthest_point



#Getting Contours which contain centre point within it
def find_contours(contours_in_func,centre_in_func,img):
    shortlist=[]
    for i in contours_in_func:
        x, y, w, h = cv2.boundingRect(i)
        if (centre_in_func[0]<(x+w) and centre_in_func[0]>x) and (centre_in_func[1]>y and centre_in_func[1]<y+h):
            shortlist.append(i)
    cv2.drawContours(img,shortlist,-1,(255,0,0),5)
    return shortlist


def main():
    final_times = []
    img = cv2.imread('E:\\erez\\arduino\\Clock solver\\Code\\python_code\\Clock-photo5.jpg')
    img_grayscale=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img_inverse=cv2.bitwise_not(img_grayscale)

    #thresholding part
    ret,thresh = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
    kernel = np.ones((5,5),np.uint8)
    thresh=cv2.dilate(thresh,kernel,iterations=2)
    edges=cv2.Canny(thresh,100,200)
    cv2.imshow('EDGES',edges)
    cv2.imshow('THRESHOLDED IMAGE',thresh)

    gray_img = cv2.cvtColor(img,	cv2.COLOR_BGR2GRAY)
    img	= cv2.medianBlur(gray_img,	5)

    #Circle Detection Part
    circles	= cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,35,param1=param_1,param2=param_2,minRadius=min_Radius,maxRadius=max_Radius)
    circles	= np.uint16(np.around(circles))
    maxrad=0
    centre=()
 
    
    for i in range (len(circles[0])):
        if circles[0,:][i][2]>90 and circles[0,:][i][2] < 160:
            centre=(circles[0,:][i][0],circles[0,:][i][1])
            #Displaying Circle Around Clock
            maxrad = circles[0,:][i][2]
            cv2.circle(img,centre,maxrad,(0,255,0),2)
            cv2.circle(img,centre,2,(0,0,255),3)

            #Getting and Displaying Contours
            contours,hierarchy=cv2.findContours(edges.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

            # Define the rectangle coordinates
            x, y, w, h = centre[0]-maxrad, centre[1]-maxrad, 2*maxrad, 2*maxrad
            
            # Filter the contours based on whether they are inside the blocking square or not
            
            
            max_length = 0
            
            for cnt in contours:
                for point in cnt:
                    x, y = point[0]
                    if (x-centre[0])**2 + (y-centre[1])**2 < (maxrad-10)**2  :
                        if (x-centre[0])**2 + (y-centre[1])**2 > max_length :
                            max_length = (x-centre[0])**2 + (y-centre[1])**2
                            final_x,final_y = point[0]
                            x, y, w, h = cv2.boundingRect(cnt)
                            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)

            final_point = (final_x,final_y)
            

            cv2.circle(img,(final_point[0],final_point[1]), 10, (0,0,255), -1)
            rad_angle = get_angle(final_point,centre)
            hour_angle=np.rad2deg(rad_angle)


            hour=get_hour(hour_angle)

            final_times.append((centre[0],centre[1],hour))

            time_display="The time is: "+str(hour)


            cv2.putText(img,time_display, (centre[0]-150,centre[1]+50), font, 1, (0, 0, 255), 2, cv2.LINE_AA)

    
    final_times =sorted(final_times, key=lambda x: x[1])
    final_times = [final_times[i:i+3] for i in range(0, len(final_times), 3)]
    final_times = [sorted(lst, key=lambda x: x[0]) for lst in final_times]
    Clock = [item[2] for i in final_times for item in i]

    print(Clock)
    cv2.imshow('Final',img)

    cv2.waitKey(0)
main()
