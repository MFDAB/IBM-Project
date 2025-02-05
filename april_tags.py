import robotpy_apriltag
import cv2
import math
import time
from djitellopy import tello
def detect(frame , me):
    def get_apriltag_detector_and_estimator(frame_size):
        detector = robotpy_apriltag.AprilTagDetector()
        assert detector.addFamily("tag36h11")
        tagConfig = robotpy_apriltag.AprilTagPoseEstimator.Config(0.2, 500, 500, frame_size[1]/2.0, frame_size[0]/2.0)
        estimator = robotpy_apriltag.AprilTagPoseEstimator(tagConfig)
        return detector, estimator

    def process_apriltag(estimator, tag):
        tag_id = tag.getId()
        center = tag.getCenter()
        hamming = tag.getHamming()
        decision_margin = tag.getDecisionMargin()
        # print("Hamming for {} is {} with decision margin {}".format(tag_id, hamming, decision_margin))
        est = estimator.estimateOrthogonalIteration(tag, 50)
        return tag_id, est.pose1, center

    def draw_tag(frame, result):
        assert frame is not None
        assert result is not None
        tag_id, pose, center = result
        print(center)
        center_x = center.x
        center_y = center.y
        cv2.circle(frame, (int(center.x), int(center.y)), 50, (255, 0, 255), 3)
        msg = f"Tag ID: {tag_id} Pose: {pose}"
        cv2.putText(frame, msg, (100, 50*1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255),2)
        return frame, tag_id, center_x, center_y

    def detect_and_process_apriltag(frame, detector, estimator):
        center_x = 0
        center_y = 0
        assert frame is not None
        tag_id = -1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tag_info = detector.detect(gray)
        DETECTION_MARGIN_THRESHOLD = 100
        filter_tags = [tag for tag in tag_info if tag.getDecisionMargin() > DETECTION_MARGIN_THRESHOLD]
        results = [ process_apriltag(estimator, tag) for tag in filter_tags ]
        for result in results:
            frame, tag_id, center_x, center_y = draw_tag(frame, result)

        return frame, tag_id, center_x, center_y

    def get_send_rc_distance(center_x, center_y , me):
        drone_height = me.get_height()
        image_center = (240, 180)
        FOV = 82.6
        focal_length = 120/math.tan(math.tan(FOV/2))
        go_xyz_x = -(center_x - image_center[0]) #*((drone_height-focal_length)/focal_length)
        go_xyz_y = (center_y - image_center[1]) #*((drone_height-focal_length)/focal_length)
        distance = math.sqrt(go_xyz_x**2 + go_xyz_y**2)
        print(f"distance:{distance}")
        # print(f"send_rc_x: {go_xyz_x}, send_rc_y: {go_xyz_y}")
        return go_xyz_x, go_xyz_y, distance

    # while True:
    WIDTH = 480 
    HEIGHT = 360

    imgContour = frame.copy()
    detector, estimator = get_apriltag_detector_and_estimator((HEIGHT, WIDTH))
    frame_with_maybe_apriltags, tag_id, center_x, center_y = detect_and_process_apriltag(imgContour, detector, estimator)
    go_xyz_x, go_xyz_y, distance = get_send_rc_distance(center_x, center_y , me)

       # capture_window_name = 'Capture Window'
       # cv2.imshow(capture_window_name, frame_with_maybe_apriltags)
       # if cv2.waitKey(1) & 0xFF == ord('q'):
            #break
    return tag_id, go_xyz_x, go_xyz_y, distance

def move_to(AprID , me , frame):
    tag_id, go_xyz_x, go_xyz_y, distance = detect(frame , me)
    print(f"go to AprilTag {tag_id}")
    if tag_id == AprID:
        sign_x = go_xyz_x / 10#abs(go_xyz_x)
        sign_y = go_xyz_y / 10#abs(go_xyz_y)
        print(f"sing_x: {sign_x}, sign_y: {sign_y}")
        print(f"rc 1: {int(sign_x)*1}")
        print(f"rc 2: {int(sign_y)*1}")
        me.send_rc_control(int(sign_x)*-1, int(sign_y)*-1, 0, 0) # code error
        time.sleep(1)