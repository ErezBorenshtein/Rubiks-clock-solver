import cv2
import os

def main():
    # Open a camera device (0 is typically the default camera)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    # Directory to save captured images
    save_dir = 'captured_images'
    os.makedirs(save_dir, exist_ok=True)
    img_counter = 0
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # Display the captured frame
        cv2.imshow('Camera Feed', frame)
        
        # Check for user input to quit
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):  # Quit when 'q' is pressed
            break
        elif key & 0xFF == ord(' '):  # Save image when spacebar is pressed
            img_name = os.path.join(save_dir, f"captured_image_{img_counter}.jpg")
            cv2.imwrite(img_name, frame)
            print(f"Saved {img_name}")
            img_counter += 1
    
    # Release the camera and close any open windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
