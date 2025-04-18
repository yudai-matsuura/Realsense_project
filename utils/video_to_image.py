import cv2
import os
 
video_path = "input_video.mp4"
output_dir = "output_images"

os.makedirs(output_dir, exist_ok = True)

cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("動画を開けませんでした．パスを確認してください")
    exit()

frame_count = 0

display_width = 1024

while True:
    ret, frame = cap.read()

    if not ret:
        print("動画の再生が終了しました")
        break

    height, width = frame.shape[:2]

    resize_ratio = display_width / width
    new_height = int(height * resize_ratio)
    resized_frame = cv2.resize(frame,(display_width, new_height))

    cv2.imshow("Video Frame", resized_frame)

    key = cv2.waitKey(1) & 0xFF

    # if space key pressed
    if key == ord(' '):
        frame_filename = os.path.join(output_dir, f"frame_{frame_count:04d}.png")
        cv2.imwrite(frame_filename, frame)
        print(f"フレームを保存しました: {frame_filename}")
    # press q to quit
    if key == ord("q"):
        print("終了しました")
        break

    frame_count += 1

cap.release()
cv2.destroyAllWindows()