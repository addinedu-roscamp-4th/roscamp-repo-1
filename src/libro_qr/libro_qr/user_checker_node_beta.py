#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from picamera2 import Picamera2
import cv2
from pyzbar.pyzbar import decode
import time
import os
from ament_index_python.packages import get_package_share_directory
from PIL import Image, ImageSequence, ImageDraw, ImageFont
import threading
from .pinky_lcd import LCD

class UserCheckerNode(Node):
    def __init__(self):
        super().__init__('user_checker_node')
        self.declare_parameter('robot_id', 'libro1')  # 기본값으로 libro1 설정 / libro2 , libro3 .....
        self.robot_id = self.get_parameter('robot_id').value
        
        # Create subscriber for user_id
        self.subscription = self.create_subscription(
            Int32,
            f'{self.robot_id}/check_user',
            self.user_check_callback,
            10)
        
        # Create subscriber for emergency_stop
        self.emergency_stop_subscription = self.create_subscription(
            Bool,
            f'{self.robot_id}/stop',
            self.emergency_stop_callback,
            10)
        
        # Create publisher for check result
        self.publisher = self.create_publisher(
            Bool,
            f'{self.robot_id}/check_user_info',
            10)
        
        # Initialize LCD
        try:
            self.lcd = LCD()
            self.lcd.set_backlight(50)
            self.get_logger().info('LCD initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize LCD: {str(e)}')
            raise
        
        # Get emotion path
        try:
            package_share_directory = get_package_share_directory('libro_qr')
            self.emotion_path = os.path.join(package_share_directory, 'emotion')
            self.get_logger().info(f'Emotion path: {self.emotion_path}')
        except Exception as e:
            # Fallback to relative path if package share directory not found
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.emotion_path = os.path.join(current_dir, '..', 'emotion')
            self.get_logger().warn(f'Using fallback emotion path: {self.emotion_path}')
        
        # Initialize camera for Pinky robot
        try:
            self.picam2 = Picamera2()
            # Configure camera for Pinky's Picamera
            self.picam2.configure(
                self.picam2.create_preview_configuration(
                    main={"format": "RGB888", "size": (640, 480)},
                    lores={"size": (320, 240), "format": "YUV420"}
                )
            )
            self.get_logger().info('Pinky Picamera initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Pinky Picamera: {str(e)}')
            raise
        
        # Display control
        self.display_active = True
        self.display_thread = threading.Thread(target=self.display_basic_emotion, daemon=True)
        self.display_thread.start()
        
        self.get_logger().info('Pinky User Checker Node initialized successfully')
        
    def display_basic_emotion(self):
        """Display basic.gif in a loop when idle"""
        while self.display_active:
            try:
                if self.display_active:
                    self.play_gif(os.path.join(self.emotion_path, "basic.gif"))
            except Exception as e:
                self.get_logger().error(f'Error displaying basic emotion: {str(e)}')
                time.sleep(1)
        
    def play_gif(self, gif_path):
        """Play a GIF file on LCD"""
        try:
            if not os.path.exists(gif_path):
                self.get_logger().error(f'GIF file not found: {gif_path}')
                return
            
            img = Image.open(gif_path)
            for frame in ImageSequence.Iterator(img):
                if not self.display_active:
                    break
                self.lcd.img_show(frame)
                time.sleep(0.1)  # Adjust frame rate
        except Exception as e:
            self.get_logger().error(f'Error playing GIF {gif_path}: {str(e)}')
    
    def show_text_message(self, message, duration=3, font_size=30):
        """Show text message on LCD for specified duration using PIL"""
        try:
            # LCD dimensions
            img_width, img_height = 320, 240
            background_color = (0, 0, 0)  # Black background
            text_color = (0, 255, 0)  # Green text
            
            # Create image
            img = Image.new('RGB', (img_width, img_height), color=background_color)
            draw = ImageDraw.Draw(img)
            
            # Try to load custom font, fallback to default
            try:
                # Look for MaruBuri-SemiBold.ttf in current directory or package directory
                font_paths = [
                    "MaruBuri-SemiBold.ttf",
                    os.path.join(os.path.dirname(__file__), "MaruBuri-SemiBold.ttf"),
                    os.path.join(self.emotion_path, "..", "MaruBuri-SemiBold.ttf")
                ]
                
                font = None
                for font_path in font_paths:
                    if os.path.exists(font_path):
                        font = ImageFont.truetype(font_path, font_size)
                        self.get_logger().info(f'Loaded font from: {font_path}')
                        break
                
                if font is None:
                    raise FileNotFoundError("Custom font not found")
                    
            except Exception as e:
                self.get_logger().warn(f'Could not load custom font: {str(e)}, using default font')
                font = ImageFont.load_default()
            
            # Calculate text position (center of image)
            bbox = draw.textbbox((0, 0), message, font=font)
            text_width = bbox[2] - bbox[0]
            text_height = bbox[3] - bbox[1]
            x = (img_width - text_width) // 2
            y = (img_height - text_height) // 2
            
            # Draw text on image
            draw.text((x, y), message, fill=text_color, font=font)
            
            # Display image on LCD
            self.lcd.img_show(img)
            time.sleep(duration)
            
        except Exception as e:
            self.get_logger().error(f'Error showing text message: {str(e)}')
    
    def display_sequence(self, text_message, gif_name, text_duration=3, gif_duration=5):
        """Display text message followed by GIF"""
        # Stop basic emotion display
        self.display_active = False
        time.sleep(0.2)  # Give time for basic display to stop
        
        try:
            # Show text message
            self.show_text_message(text_message, text_duration)
            
            # Play emotion GIF for specified duration
            gif_path = os.path.join(self.emotion_path, f"{gif_name}.gif")
            start_time = time.time()
            while time.time() - start_time < gif_duration:
                if os.path.exists(gif_path):
                    img = Image.open(gif_path)
                    for frame in ImageSequence.Iterator(img):
                        if time.time() - start_time >= gif_duration:
                            break
                        self.lcd.img_show(frame)
                        time.sleep(0.1)
                else:
                    self.get_logger().error(f'GIF file not found: {gif_path}')
                    break
            
        except Exception as e:
            self.get_logger().error(f'Error in display sequence: {str(e)}')
        finally:
            # Resume basic emotion display
            self.display_active = True
            if not self.display_thread.is_alive():
                self.display_thread = threading.Thread(target=self.display_basic_emotion, daemon=True)
                self.display_thread.start()

    def user_check_callback(self, msg):
        """Callback function when receiving user_id"""
        user_id = msg.data
        self.get_logger().info(f'Received user_id: {user_id}')
        
        # Stop basic emotion and show "Show User QR Code!" message
        self.display_active = False
        time.sleep(0.2)
        self.show_text_message("Show User QR Code!", duration=2)
        
        # Start camera
        try:
            self.picam2.start()
        except Exception as e:
            self.get_logger().error(f'Failed to start camera: {str(e)}')
            result = Bool()
            result.data = False
            self.publisher.publish(result)
            self.display_active = True
            return
        
        try:
            # Try to scan QR code for 10 seconds
            start_time = time.time()
            qr_detected = False
            
            while time.time() - start_time < 10:  # 10 seconds timeout
                try:
                    # Capture frame
                    frame = self.picam2.capture_array()
                    
                    # Convert to RGB and rotate for Pinky's camera orientation
                    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    rotate_frame = cv2.rotate(rgb_frame, cv2.ROTATE_180)
                    flipped_frame = cv2.flip(rotate_frame, 1)
                    
                    # Decode QR code
                    decoded_objects = decode(flipped_frame)
                    
                    for obj in decoded_objects:
                        qr_data = obj.data.decode("utf-8")
                        scanned_id = qr_data.split('/')[-1]  # Get the last part of URL
                        
                        # Compare IDs
                        result = Bool()
                        result.data = str(user_id) == scanned_id
                        
                        # Publish result
                        self.publisher.publish(result)
                        self.get_logger().info(f'QR Code scanned. User ID: {scanned_id}, Expected: {user_id}, Result: {result.data}')
                        
                        qr_detected = True
                        
                        # Show appropriate message and emotion based on result
                        if result.data:
                            # Correct user
                            self.display_sequence("User Detected!", "fun", 3, 5)
                        else:
                            # Different user
                            self.display_sequence("Different User", "bored", 3, 5)
                        
                        # Stop camera and return
                        self.picam2.stop()
                        return
                    
                except Exception as e:
                    self.get_logger().warn(f'Error during frame capture: {str(e)}')
                    continue
                
                time.sleep(0.1)  # Small delay to prevent high CPU usage
            
            # If no QR code was scanned within timeout
            if not qr_detected:
                result = Bool()
                result.data = False
                self.publisher.publish(result)
                self.get_logger().info('No QR code scanned within timeout')
                
                # Show timeout message and angry emotion
                self.display_sequence("No User Detected", "angry", 3, 5)
            
        finally:
            # Ensure camera is stopped
            try:
                self.picam2.stop()
            except Exception as e:
                self.get_logger().error(f'Error stopping camera: {str(e)}')

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.display_active = False
        if hasattr(self, 'lcd'):
            try:
                self.lcd.close()
            except Exception as e:
                self.get_logger().error(f'Error closing LCD: {str(e)}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    user_checker_node = UserCheckerNode()
    
    try:
        rclpy.spin(user_checker_node)
    except KeyboardInterrupt:
        pass
    finally:
        user_checker_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
