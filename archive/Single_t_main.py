import cv2

# Global/shared mouse state
mouse_state = {
    "x": 0,
    "y": 0,
    "clicked": False
}

def mouse_callback(event, x, y, flags, param):
    mouse_state["x"] = x
    mouse_state["y"] = y

    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_state["clicked"] = True
    elif event == cv2.EVENT_LBUTTONUP:
        mouse_state["clicked"] = False


def main():
    cap = cv2.VideoCapture(0)
    window = "Camera"
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, mouse_callback)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # -------------------------------
        # Example: overlay a small circle with the mouse
        # -------------------------------
        cv2.circle(frame, (mouse_state["x"], mouse_state["y"]), 5, (0, 0, 255), -1)

        # -------------------------------
        # Example: React to clicks
        # -------------------------------
        if mouse_state["clicked"]:
            cv2.putText(frame, "CLICK!", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 0), 2)

        # -------------------------------
        # Here is where you'd add your ArUco detection
        # Use `mouse_state` inside your ArUco logic
        # -------------------------------

        cv2.imshow(window, frame)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()