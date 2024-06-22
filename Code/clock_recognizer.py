import cv2

def main():
    # Open a camera device (0 is typically the default camera)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # Display the captured frame
        cv2.imshow('Camera Feed', frame)
        
        # Check for user input to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the camera and close any open windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
