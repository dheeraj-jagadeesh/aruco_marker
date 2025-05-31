import cv2
import numpy as np
import sys
import json

def load_config(config_path):
    with open(config_path, 'r') as f:
        return json.load(f)

ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def detect_aruco_tags(frame, aruco_dict_name):
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[aruco_dict_name])
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is not None and len(ids) >= 3:
        return corners, ids
    return None, None

def order_corners(corners):
    centers = [np.mean(corner[0], axis=0) for corner in corners]
    centers = np.array(centers)
    sum_coords = centers.sum(axis=1)
    diff_coords = centers[:, 0] - centers[:, 1]
    top_left_idx = np.argmin(sum_coords)
    top_right_idx = np.argmax(diff_coords)
    bottom_left_idx = np.argmax(centers[:,1])
    return [corners[top_left_idx], corners[top_right_idx], corners[bottom_left_idx]]

def get_board_homography(corners, width, height):
    src_pts = np.array([np.mean(corner[0], axis=0) for corner in corners], dtype=np.float32)
    tl, tr, bl = src_pts
    br = tr + (bl - tl)
    src = np.array([tl, tr, br, bl], dtype=np.float32)
    dst = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(dst, src)
    return H, width, height

def get_scaled_square_size(corners, ref_board_height, ref_square_fraction, min_square_size):
    pts = [np.mean(corner[0], axis=0) for corner in corners]
    board_height = np.linalg.norm(pts[0] - pts[2])
    board_width = np.linalg.norm(pts[0] - pts[1])
    base_square_size = int(min(board_width, board_height) * ref_square_fraction)
    base_square_size = max(min_square_size, base_square_size)
    scale_factor = board_height / ref_board_height
    scaled_square_size = int(base_square_size * scale_factor)
    scaled_square_size = max(min_square_size, scaled_square_size)
    return scaled_square_size

def draw_aligned_square_homography(img, center_board, square_size, H, color, thickness=2):
    half = square_size / 2
    board_pts = np.array([
        [center_board[0] - half, center_board[1] - half],
        [center_board[0] + half, center_board[1] - half],
        [center_board[0] + half, center_board[1] + half],
        [center_board[0] - half, center_board[1] + half]
    ], dtype=np.float32).reshape(-1,1,2)
    img_pts = cv2.perspectiveTransform(board_pts, H)
    img_pts = img_pts.astype(int)
    cv2.polylines(img, [img_pts], isClosed=True, color=color, thickness=thickness)

def main():
    config = load_config('config.json')
    board_width = config['board_width']
    board_height = config['board_height']
    ref_board_height = config['reference_board_height']
    ref_square_fraction = config['reference_square_fraction']
    min_square_size = config['min_square_size']
    aruco_dict_name = config['aruco_dict']
    OBJECT_RELATIVE_POSITIONS = config['object_relative_positions']
    output_video = config['output_video']

    if len(sys.argv) > 1:
        video_path = sys.argv[1]
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"Error: Cannot open video file {video_path}")
            return
        print(f"Processing video file: {video_path}")
    else:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: Cannot open webcam")
            return
        print("Using webcam input.")

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0 or fps is None:
        fps = 30

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_video, fourcc, fps, (frame_width, frame_height))

    print("Show the maintenance board with 3 ArUco tags at top-left, top-right, bottom-left corners.")
    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        corners, ids = detect_aruco_tags(frame, aruco_dict_name)
        display = frame.copy()

        if corners is not None and len(corners) >= 3:
            three_corners = order_corners(corners[:3])
            square_size = get_scaled_square_size(
                three_corners, ref_board_height, ref_square_fraction, min_square_size
            )
            H, width, height = get_board_homography(three_corners, board_width, board_height)
            tag_board_coords = [
                (0, 0), (width, 0), (0, height)
            ]
            for pt in tag_board_coords:
                draw_aligned_square_homography(display, pt, square_size, H, (0,255,0), thickness=2)
            board_outline = np.array([
                [0,0],[width,0],[width,height],[0,height]
            ], dtype=np.float32).reshape(-1,1,2)
            img_outline = cv2.perspectiveTransform(board_outline, H).astype(int)
            cv2.polylines(display, [img_outline], isClosed=True, color=(255,255,0), thickness=2)
            for rel_x, rel_y in OBJECT_RELATIVE_POSITIONS:
                board_x = rel_x * width
                board_y = rel_y * height
                draw_aligned_square_homography(display, (board_x, board_y), square_size, H, (0,0,255), thickness=2)
            cv2.putText(display, f"Dict: {aruco_dict_name}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display, "Waiting for 3 ArUco tags...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Maintenance Board Detection", display)
        out.write(display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
