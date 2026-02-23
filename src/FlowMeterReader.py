import cv2 as cv
import numpy as np
from qreader import QReader
import argparse
import time
import json
import paho.mqtt.client as mqtt
import requests
import logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s: %(message)s')


def does_range_overlap(startX1, endX1, startX2, endX2):
    if startX1 <= startX2:
        first_range_end = endX1
        second_range_start = startX2
    else:
        first_range_end = endX2
        second_range_start = startX1
    return second_range_start <= first_range_end


def midpoint(points):
    sumX = 0
    sumY = 0
    for point in points:
        sumX = point[0] + sumX
        sumY = point[1] + sumY
    return (sumX / len(points), sumY / len(points))


def findAngle(point1, point2):
    yDiff = abs(point1[1] - point2[1])
    xDiff = abs(point1[0] - point2[0])
    return np.arctan(yDiff / xDiff)


def crop(img, lowerLeftP1, upperRightP2):
    return np.copy(
        img[
            int(lowerLeftP1[1]) : int(upperRightP2[1]),
            int(lowerLeftP1[0]) : int(upperRightP2[0]),
        ]
    )


def getGuagesByQRCodes(img):
    guages = []
    qreader = QReader()
    qrcodes = qreader.detect(img)
    qrCodeInfos = []
    id = 0
    if len(qrcodes) != 6:
        raise Exception("Less than 6 qr codes detected")
    else:
        logging.debug("Found 6 QR guages")
    for qrcode in qrcodes:
        qrcodeCoords = qrcode["quad_xy"]
        (
            midPointAprox,
            lowerLeftPoint,
            lowerRightPoint,
            upperLeftPoint,
            upperRightPoint,
            isBottom,
        ) = identifyMatrix(qrcodeCoords)
        qrCodeInfo = {
            "coords": qrcodeCoords,
            "id": id,
            "lowerLeftPoint": lowerLeftPoint,
            "lowerRightPoint": lowerRightPoint,
            "upperLeftPoint": upperLeftPoint,
            "upperRightPoint": upperRightPoint,
            "isBottom": isBottom,
        }
        qrCodeInfos.append(qrCodeInfo)
        id = id + 1
    idsAssigned = set()
    for qrCodeInfo in qrCodeInfos:
        if qrCodeInfo["id"] in idsAssigned:
            continue
        idsAssigned.add(qrCodeInfo["id"])
        guage = {}
        if qrCodeInfo["isBottom"]:
            guage["bottomQr"] = qrCodeInfo
        else:
            guage["topQr"] = qrCodeInfo
        for matchingQrCodeInfo in qrCodeInfos:
            if matchingQrCodeInfo["id"] in idsAssigned:
                continue
            if does_range_overlap(
                matchingQrCodeInfo["lowerLeftPoint"][0],
                matchingQrCodeInfo["lowerRightPoint"][0],
                qrCodeInfo["lowerLeftPoint"][0],
                qrCodeInfo["lowerRightPoint"][0],
            ):
                if matchingQrCodeInfo["isBottom"]:
                    guage["bottomQr"] = matchingQrCodeInfo
                else:
                    guage["topQr"] = matchingQrCodeInfo
                idsAssigned.add(matchingQrCodeInfo["id"])
                break
        if "topQr" not in guage or "bottomQr" not in guage:
            raise Exception(
                f"One of the guges {guage} from assigned {idsAssigned} did not have a bottom or top QR: {qrCodeInfos}"
            )
        guages.append(guage)
        # Sort gauges by the average x-coordinate of their top and bottom QR codes to ensure left-to-right order
        guages.sort(
            key=lambda g: (
                (
                    g["topQr"]["upperLeftPoint"][0]
                    + g["topQr"]["upperRightPoint"][0]
                    + g["bottomQr"]["lowerLeftPoint"][0]
                    + g["bottomQr"]["lowerRightPoint"][0]
                )
                / 4
            )
        )
    return guages


def identifyMatrix(qrPoints):
    midPointAprox = midpoint(qrPoints)
    lowerLeftPoint = None
    upperRightPoint = None
    lowerRightPoint = None
    upperLeftPoint = None
    isBottom = qrPoints[0][1] > 500
    for qrPoint in qrPoints:
        if qrPoint[0] > midPointAprox[0]:
            # right
            if qrPoint[1] < midPointAprox[1]:
                # top
                upperRightPoint = qrPoint
            else:
                # bottom
                lowerRightPoint = qrPoint
        else:
            # left
            if qrPoint[1] < midPointAprox[1]:
                # top
                upperLeftPoint = qrPoint
            else:
                # bottom
                lowerLeftPoint = qrPoint
    return (
        midPointAprox,
        lowerLeftPoint,
        lowerRightPoint,
        upperLeftPoint,
        upperRightPoint,
        isBottom,
    )


def getGuageTracks(guages):
    guageTracks = []
    for guage in guages:
        guageTracks.append(
            {
                "ids": [guage["topQr"]["id"], guage["bottomQr"]["id"]],
                "topPoint": (
                    int(
                        (
                            guage["topQr"]["upperLeftPoint"][0]
                            + guage["topQr"]["upperRightPoint"][0]
                        )
                        / 2
                    ),
                    int(
                        max(
                            guage["topQr"]["upperLeftPoint"][1],
                            guage["topQr"]["upperRightPoint"][1],
                        )
                    ),
                ),
                "bottomPoint": (
                    int(
                        (
                            guage["bottomQr"]["lowerLeftPoint"][0]
                            + guage["bottomQr"]["lowerRightPoint"][0]
                        )
                        / 2
                    ),
                    int(
                        max(
                            guage["bottomQr"]["lowerLeftPoint"][1],
                            guage["bottomQr"]["lowerRightPoint"][1],
                        )
                    ),
                ),
                "yTop": max(
                    guage["topQr"]["lowerLeftPoint"][1],
                    guage["topQr"]["lowerRightPoint"][1],
                )
                + 10,
                "yBottom": min(
                    guage["bottomQr"]["upperLeftPoint"][1],
                    guage["bottomQr"]["upperRightPoint"][1],
                )
                - 10,
            }
        )
    return guageTracks


def indicatorSizeUsingEdge(startX, endX, yMid, edges):
    startIdx = -1  # noqa: F841
    endIdx = -1  # noqa: F841
    indicatorSize = 0
    xCoords = startX
    while xCoords < endX and xCoords < len(edges[0]):
        if edges[int(yMid)][int(xCoords)] != 0:
            if startIdx == -1:
                startIdx = xCoords
            else:
                endIdx = xCoords
        xCoords = xCoords + 1
        if startIdx == -1 or endIdx == -1:
            indicatorSize = 0
        else:
            indicatorSize = endIdx - startIdx
    return indicatorSize


def indicatorSizeUsingContinuous(startX, endX, yMid, bit_not):
    startIdx = -1  # noqa: F841
    endIdx = -1  # noqa: F841
    indicatorSize = 0
    xCoords = startX
    while xCoords < endX and xCoords < len(bit_not[0]):
        if bit_not[int(yMid)][int(xCoords)] != 0:
            if startIdx == -1:
                startIdx = xCoords
            else:
                endIdx = xCoords
        else:
            if endIdx - startIdx > indicatorSize:
                indicatorSize = endIdx - startIdx
            startIdx = -1  # noqa: F841
            endIdx = -1  # noqa: F841
        xCoords = xCoords + 1
    return indicatorSize


def indicatorSizeUsingVerticalDistance(startX, endX, yMid, testHeight, bit_not):
    indicatorSize = 0
    xCoords = startX
    while xCoords < endX and xCoords < len(bit_not[0]):
        if bit_not[int(yMid)][int(xCoords)] != 0:
            height = yMid
            while height < yMid + testHeight:
                if bit_not[int(height)][int(xCoords)] == 0:
                    break
                height += 1
            if height - yMid > indicatorSize:
                indicatorSize = height - yMid
        xCoords = xCoords + 1
    return indicatorSize


def getXCoord(yCoord, slope, intercept):
    return (yCoord - intercept) / slope


def processFrame(img, aproxIndicatorLength=600):
    img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    blur = cv.GaussianBlur(img, (11, 11), 0)
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    lower_silver = np.array([0, 0, 180])
    upper_silver = np.array([255, 255, 255])
    mask = cv.inRange(hsv, lower_silver, upper_silver)

    guages = getGuagesByQRCodes(img)
    guageTracks = getGuageTracks(guages)
    logging.debug(f"Got gauge tracks {len(guageTracks)}")
    for guageTrack in guageTracks:
        coefficients = np.polyfit(
            [guageTrack["bottomPoint"][0], guageTrack["topPoint"][0]],
            [guageTrack["bottomPoint"][1], guageTrack["topPoint"][1]],
            1,
        )
        slope, intercept = coefficients
        yMid = guageTrack["yTop"]
        yMax = guageTrack["yBottom"]
        guageIndicatorSizes = []
        averageIndicatorSum = 0
        averageIndicatorCount = 0
        while (
            yMid < guageTrack["bottomPoint"][1] + aproxIndicatorLength and yMid < yMax
        ):
            xMid = getXCoord(yMid, slope, intercept)
            xCoords = xMid - 30
            indicatorSize = indicatorSizeUsingVerticalDistance(
                xCoords, xMid + 30, yMid, 50, mask
            )
            if indicatorSize > 0:
                guageIndicatorSizes.append((int(xMid + 50), int(yMid), indicatorSize))
                averageIndicatorSum = averageIndicatorSum + indicatorSize
                averageIndicatorCount = averageIndicatorCount + 1
            yMid = yMid + 1
        averageIndicatorAverage = averageIndicatorSum / averageIndicatorCount
        indicatedPixel = 0
        indicatedWidth = 0
        for guageIndicatorSize in guageIndicatorSizes:
            if guageIndicatorSize[2] > averageIndicatorAverage:
                if guageIndicatorSize[2] > indicatedWidth:
                    indicatedPixel = guageIndicatorSize[1]
                    indicatedWidth = guageIndicatorSize[2]
        percent = ((indicatedPixel - guageTrack["yBottom"]) * -100) / (
            guageTrack["yBottom"] - guageTrack["yTop"]
        )
        guageTrack["percent"] = percent
    return guageTracks


def main():
    parser = argparse.ArgumentParser(description="FlowMeterReader HTTP Image to MQTT")
    parser.add_argument(
        "--url",
        required=True,
        help="HTTP URL to fetch JPEG image (e.g., http://192.168.49.171/image.jpg)",
    )
    parser.add_argument(
        "--username", default=None, help="Basic auth username for HTTP image URL"
    )
    parser.add_argument(
        "--password", default=None, help="Basic auth password for HTTP image URL"
    )
    parser.add_argument(
        "--interval", type=float, default=1.0, help="Interval between frames in seconds"
    )
    parser.add_argument("--broker", required=True, help="MQTT broker address")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument(
        "--topic", required=True, help="MQTT topic to publish gauge percentages"
    )
    args = parser.parse_args()
    logging.debug(f"Parsed arguments: {args}")
    # Logging is configured at the top of the file

    client = mqtt.Client()
    client.connect(args.broker, args.port, 60)
    logging.debug(f"Connected to MQTT broker {args.broker}:{args.port}")
    client.loop_start()

    def fetch_image():
        auth = (
            (args.username, args.password) if args.username and args.password else None
        )  # noqa: F841
        try:
            response = requests.get(args.url, auth=auth, timeout=5)
            response.raise_for_status()
            img_array = np.frombuffer(response.content, np.uint8)
            img = cv.imdecode(img_array, cv.IMREAD_COLOR)
            return img
        except Exception as e:
            logging.error(f"Failed to fetch image: {e}")
            return None

    # Initial fetch to verify URL
    frame = fetch_image()
    if frame is None:
        logging.error("Failed to retrieve initial image")
        return

    try:
        while True:
            frame = fetch_image()
            if frame is None:
                logging.warning("Failed to fetch image")
                break
            guage_tracks = processFrame(frame)
            logging.debug(f"Processed frame, obtained {len(guage_tracks)} gauge tracks")
            payload = json.dumps(
                [
                    {"id": i, "percent": str(gt.get("percent", None))}
                    for i, gt in enumerate(guage_tracks)
                ]
            )
            logging.info(
                f"{str(round(guage_tracks[0].get('percent', None), 2))},{str(round(guage_tracks[1].get('percent', None), 2))},{str(round(guage_tracks[2].get('percent', None), 2))}"
            )
            client.publish(args.topic, payload)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        logging.info("Interrupted by user")
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
