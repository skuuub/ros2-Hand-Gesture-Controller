import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import mediapipe as mp
import cv2

class TurtleKeyController(Node):
    def __init__(self):
        super().__init__('turtle_key_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("TurtleKeyController Node started. Use hand gestures to move the turtle.")

        self.mvt_cmd = "no movement"
        
        # Initialize MediaPipe for hand tracking
        self.mpHands = mp.solutions.hands
        self.hand = self.mpHands.Hands(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.8,
            max_num_hands=1
        )

        self.run()

    def run(self):
        # Initialize webcam
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("Could not open the webcam.")
            return

        draw = mp.solutions.drawing_utils
        
        # Store distances or positions
        thumb_x1 = thumb_y1 = 0
        pointer_x1 = pointer_y1 = 0
        middle_x1 = middle_y1 = 0
        ring_x1 = ring_y1 = 0
        pinkie_x1 = pinkie_y1 = 0
        
        twist = Twist()

        try:
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().error("Failed to read frame from webcam.")
                    break

                frame = cv2.flip(frame, 1)
                image_height, image_width, _ = frame.shape
                frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                processed = self.hand.process(frameRGB)

                # -------------------------------------------------------------
                # Only update mvt_cmd if we actually detect a hand
                # -------------------------------------------------------------
                if processed.multi_hand_landmarks:
                    hand_landmarks = processed.multi_hand_landmarks[0]
                    draw.draw_landmarks(frame, hand_landmarks, self.mpHands.HAND_CONNECTIONS)

                    for id, lm in enumerate(hand_landmarks.landmark):
                        x = int(lm.x * image_width)
                        y = int(lm.y * image_height)

                        if id == 4:  # Thumb
                            thumb_x1 = x
                            thumb_y1 = y
                        elif id == 8:  # Pointer
                            pointer_x1 = x
                            pointer_y1 = y
                        elif id == 12:  # Middle
                            middle_x1 = x
                            middle_y1 = y
                        elif id == 16:  # Ring
                            ring_x1 = x
                            ring_y1 = y
                        elif id == 20:  # Pinkie
                            pinkie_x1 = x
                            pinkie_y1 = y

                    # Calculate distances between thumb and other fingers
                    move_forward_dist = abs(pointer_y1 - thumb_y1)
                    move_left_dist    = abs(middle_y1  - thumb_y1)
                    move_right_dist   = abs(ring_y1    - thumb_y1)
                    move_back_dist    = abs(pinkie_y1  - thumb_y1)

                    # Determine gesture by checking thresholds
                    if move_forward_dist != 0 and move_forward_dist < 30:
                        self.mvt_cmd = "move forward"
                    elif move_left_dist != 0 and move_left_dist < 30:
                        self.mvt_cmd = "move left"
                    elif move_right_dist != 0 and move_right_dist < 30:
                        self.mvt_cmd = "move right"
                    elif move_back_dist != 0 and move_back_dist < 30:
                        self.mvt_cmd = "move back"
                    else:
                        # If a hand is present, but none of the distances
                        # are less than 30, then set "no movement".
                        self.mvt_cmd = "no movement"

                    self.get_logger().info(f"Current gesture: {self.mvt_cmd}")
                else:
                    # If no hand is detected, DO NOT reset self.mvt_cmd.
                    # Instead, we leave it *as is*. This means the turtle
                    # will continue the last recognized command indefinitely
                    # until a new hand/gesture is seen.
                    #
                    # Example:
                    # pass
                    pass

                # -------------------------------------------------------------
                # Now publish the twist according to the *current* self.mvt_cmd
                # -------------------------------------------------------------
                if self.mvt_cmd == 'move forward':
                    twist.linear.x = 1.0
                    twist.angular.z = 0.0
                elif self.mvt_cmd == 'move back':
                    twist.linear.x = -1.0
                    twist.angular.z = 0.0
                elif self.mvt_cmd == 'move left':
                    twist.linear.x = 0.0
                    twist.angular.z = 1.0
                elif self.mvt_cmd == 'move right':
                    twist.linear.x = 0.0
                    twist.angular.z = -1.0
                else:  # 'no movement'
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.publisher_.publish(twist)

                cv2.imshow("Frame", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        finally:
            cap.release()
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleKeyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()