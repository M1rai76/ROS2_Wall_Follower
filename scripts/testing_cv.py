import cv2

# Function to get HSV values when clicking on the image
def get_hsv_value(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Get the HSV values at the clicked point (x, y)
        hsv_value = param[y, x]  # y is row, x is column
        print(f"HSV Value at ({x}, {y}): {hsv_value}")

# Load the screenshot image
image = cv2.imread("D:\\C-Driver Files Here\\Downloads\\wall_follower\\scripts\\pink_on_blue.png")
# Check if the image was successfully loaded
if image is None:
    print("Error: Could not load the image. Check the file path.")
else:
    # Convert the image to HSV format
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Display the HSV image
    cv2.namedWindow("Screenshot (HSV)", cv2.WINDOW_NORMAL)
    cv2.imshow("Screenshot (HSV)", hsv_image)

    # Set a mouse callback to get the HSV value on click
    cv2.setMouseCallback("Screenshot (HSV)", get_hsv_value, hsv_image)

    # Wait until a key is pressed or the window is closed
    cv2.waitKey(0)
    cv2.destroyAllWindows()
