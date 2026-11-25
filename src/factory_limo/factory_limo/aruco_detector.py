"""
aruco_detector.py (WBS 2.2)

Limo 정면 카메라 이미지에서 ArUco 마커를 찾아 "이번 프레임에 보인 마커들의
목록"을 발행하는 노드. 이 노드는 "어떤 마커가 어디 있는지"만 알려주고,
그 정보로 뭘 할지(핸드오프 판단, P-제어 등)는 전혀 모른다 — 그 책임은
go_to_station_server.py(2.5)가 가진 coarse_mover/aruco_docker가 진다.
역할을 이렇게 분리해두면 카메라/캘리브레이션 쪽 문제와 제어 로직 쪽 문제를
따로 디버깅할 수 있다.

메시지 설계에 대한 솔직한 트레이드오프: 이상적으로는 factory_interfaces에
ArucoDetection[] 같은 커스텀 메시지 타입을 추가하는 게 깔끔하지만, 그러면
factory_interfaces를 다시 빌드해야 하고 이미 완성된 1.1 작업에 손을 대야 한다.
3일 일정에서 "빠르게 돌아가는 것"이 우선이라, 여기서는 std_msgs/String에 JSON을
실어 보내는 실용적인 방법을 택했다. 나중에 시간이 남으면 정식 메시지 타입으로
바꾸는 걸 권장한다 (TODO로 남겨둠).

발행 토픽: /limo/aruco_detections (std_msgs/String)
  JSON 형식: [{"id": 1, "distance_m": 0.42, "lateral_m": -0.03, "yaw_rad": 0.01}, ...]
  (이번 프레임에 아무 마커도 안 보이면 "[]")

좌표 부호 규약(카메라 기준): distance_m = 마커까지의 전방 거리(z),
lateral_m = 마커가 카메라 중심 기준 오른쪽(+)/왼쪽(-)으로 치우친 정도(x),
yaw_rad = 마커 평면과 카메라 광축 사이의 상대 회전. 실제 카메라에 장착한 뒤
부호가 반대로 나오면 아루코 docker 쪽이 아니라 여기서 부호만 뒤집으면 된다.

참고: 이 환경에는 rclpy/OpenCV(cv2.aruco)가 없어서 실제로 실행해 검증하지는
못했다. OpenCV 버전에 따라 cv2.aruco API가 달라서(4.7+ ArucoDetector 클래스 vs
구버전 cv2.aruco.detectMarkers 함수), 실제 환경에서 cv2.__version__을 먼저
확인하고 필요하면 아래 _detect_markers를 구버전 API로 바꿔야 할 수 있다.
"""

import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("marker_length_m", 0.1)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")

        self._marker_length_m = self.get_parameter("marker_length_m").value
        dict_name = self.get_parameter("aruco_dictionary").value
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self._detector_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._detector_params)

        self._bridge = CvBridge()
        self._camera_matrix = None
        self._dist_coeffs = None

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, 10)
        self.create_subscription(Image, image_topic, self._on_image, 10)
        self._pub = self.create_publisher(String, "/limo/aruco_detections", 10)

        self.get_logger().info(f"aruco_detector 시작됨 (image_topic={image_topic})")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.array(msg.d, dtype=np.float64)

    def _on_image(self, msg: Image) -> None:
        if self._camera_matrix is None:
            return  # 아직 CameraInfo를 못 받았으면 거리 추정이 불가능하니 건너뜀

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        detections = []
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self._marker_length_m, self._camera_matrix, self._dist_coeffs
            )
            for marker_id, rvec, tvec in zip(ids.flatten(), rvecs, tvecs):
                distance_m = float(tvec[0][2])
                lateral_m = float(tvec[0][0])
                rot_matrix, _ = cv2.Rodrigues(rvec[0])
                yaw_rad = float(np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0]))
                detections.append(
                    {
                        "id": int(marker_id),
                        "distance_m": distance_m,
                        "lateral_m": lateral_m,
                        "yaw_rad": yaw_rad,
                    }
                )

        out = String()
        out.data = json.dumps(detections)
        self._pub.publish(out)


def main():
    rclpy.init()
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
