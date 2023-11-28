import cv2
import math
import time


def calculate_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def find_shortest_path(points):
    """
    The find_shortest_path function implements the Nearest Neighbor heuristic by 
    iteratively selecting the nearest unvisited point from the current point until 
    all points are visited.    
    """
    num_points = len(points)
    path = [0]  # Start with the first point
    visited = [False] * num_points
    visited[0] = True

    while len(path) < num_points:
        current_point = path[-1]
        min_distance = float('inf')
        nearest_point = None

        for i in range(num_points):
            if not visited[i]:
                distance = calculate_distance(points[current_point], points[i])
                if distance < min_distance:
                    min_distance = distance
                    nearest_point = i

        path.append(nearest_point)
        visited[nearest_point] = True

    return path


def find_traverse_path(image_file, type='head'):
    img = cv2.imread(image_file)
    print("Origional Image Shape:", img.shape)
    # img = cv2.resize(img, None, fx=0.6, fy=0.6)  # Rescale the image
    # print("Rescaled shape:", img.shape)

    with open('obj.names', 'r') as f:
        classes = f.read().splitlines()
    print(classes)

    net = cv2.dnn.readNetFromDarknet('yolov4-custom.cfg', 'yolov4-custom_3000.weights')

    model = cv2.dnn_DetectionModel(net)
    model.setInputParams(scale=1 / 255, size=(416, 416), swapRB=True)

    classIds, scores, boxes = model.detect(img, confThreshold=0.6, nmsThreshold=0.4)
    
    #print(classIds)

    box_centers = {'head': [], 'hole': []}  # List to store box centers
    i = 0
    for (classId, score, box) in zip(classIds, scores, boxes):
        x, y, w, h = box  # Unpack the bounding box coordinates
        print(classId)
        # Calculate box center
        center_x = x + w // 2
        center_y = y + h // 2

        if classId == 0:
            box_centers['head'].append((center_x, center_y))

            cv2.rectangle(img, (x, y), (x + w, y + h), color=(255, 0, 0), thickness=2)
            # text = '%s: %.2f' % (classes[classId], score)
            text = '%s-(%d, %d): %.2f' % (classes[classId[0]], center_x, center_y, score)
            cv2.putText(img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(255, 0, 0), thickness=2)
        if classId == 1:
            box_centers['hole'].append((center_x, center_y))

            cv2.rectangle(img, (x, y), (x + w, y + h), color=(0, 255, 0), thickness=2)
            # text = '%s: %.2f' % (classes[classId], score)
            text = '%s-(%d, %d): %.2f' % (classes[classId[0]], center_x, center_y, score)
            cv2.putText(img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 255, 0), thickness=2)
        i += 1

    cv2.imwrite('output.jpg', img)
    # cv2.imshow('Image', img)
    # img_predictions = cv2.imread('output.jpg')
    # cv2.imshow("Image",img_predictions)

    print('Box Centers:', box_centers)

    shortest_path = find_shortest_path(box_centers[type])

    # for index in shortest_path:
    #     print(box_centers[index])

    return [box_centers[type], shortest_path]

    cv2.waitKey(0)
    cv2.destroyAllWindows()


def map_scale(input_coordinates, img_h, img_w, wp_w, wp_h):
    x = input_coordinates[0]
    y = input_coordinates[1]
    actual_x = int(round((wp_w / img_w) * x))
    actual_y = int(round((wp_h / img_h) * y))

    #print("IN=", input_coordinates, "MP=", (actual_x, actual_y))
    return (actual_x, actual_y)


def xy_transform(x, y, LX, LY):
    return (LX - x, LY + y)


def image_capture(IMG_W, IMG_H):
    cap = cv2.VideoCapture(0)
    time.sleep(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
    ret, frame = cap.read()
    cv2.imwrite('input.jpg', frame)
    image_file = 'input.jpg'
    print("Image Captured!")
    # img = cv2.imread(image_file)
    # print("Origional shape:",img.shape)

    return image_file


def find_transformed_values(mapped_point_coordinates, LX, LY, wp_thickness, a2=250, a3=320, d=360):
    transformed_coordinates = xy_transform(mapped_point_coordinates[0], mapped_point_coordinates[1], LX,
                                           LY)  # XY plane transform

    print("MP=", mapped_point_coordinates, "TF=", transformed_coordinates)

    X = transformed_coordinates[0]
    Y = transformed_coordinates[1]

    # X,Y=mapped_point_coordinates[0],mapped_point_coordinates[1]

    Z = int((1300/360)*(d - wp_thickness))
    theta2=-math.acos((X**2+Y**2-a2**2-a3**2)/(2*a3*a2))
    theta1=math.atan(Y/X)+math.atan(a3*math.sin(theta2)/(a2+a3*math.cos(theta2)))

    if X<0:
        theta2=abs(theta2)
        theta1=abs(theta1)

    theta1 = int(math.degrees(theta1))
    theta2 = int(math.degrees(theta2))-95
    #theta2_0 = math.acos((X ** 2 + Y ** 2 - a3 ** 2 - a2 ** 2) / (2 * a3 * a2))

    # theta1 = math.asin((a3 * math.sin(theta2_0)) / math.sqrt(X ** 2 + Y ** 2)) + math.atan2(Y, X)
    #theta1 = math.atan(Y / X) - (math.atan((a3 * math.sin(theta2_0)) / (a2 + a3 * math.cos(theta2_0))))

    #theta2_0 = math.degrees(math.fmod(theta2_0, 2 * math.pi))
    #theta1 = int(math.degrees(math.fmod(theta1, 2 * math.pi)))
    #theta2 = int(theta2_0)

    # theta3=math.acos((X**2-Y**2-a3**2-a2**2)/(2*a3*a2))
    # theta2=math.atan((Y*(a2+a3*math.cos(theta3))-X*a3*math.sin(theta3))/(X+a3*math.cos(theta3)+Y*a3*math.sin(theta3)))
    # print("X,Y,a2,a3=", X, Y, a2, a3)
    #print("θ1=", theta1, "θ2=", theta2, "Z=", Z)
    return theta1, theta2, Z
    # return (20, 30, Z)


def exec_get_traverse_path(type, wp_thickness):
    # CONSTANTS
    IMG_H, IMG_W, WP_W, WP_H = 720, 1280, 250, 235  # (720,1280)=>(335mm,150mm)
    LX, LY = WP_W // 2, 300  # mm
    # wp_thickness = 20
    a2, a3 = 250, 320
    d = 360

    # Get input from camera
    image_file = image_capture(IMG_W, IMG_H)

    # get the shortest traverse path
    traverse_centers, shortest_path = find_traverse_path(image_file, type)
    # print(shortest_path)
    # print(traverse_centers)

    transformed_shortest_path = []
    prev_theta1,prev_theta2,prev_Z=0,0,0
    for i in range(len(traverse_centers)):
        point=traverse_centers[i]
        mapped_point_coordinates = map_scale(point, IMG_H, IMG_W, WP_W, WP_H)

        theta1, theta2, Z = find_transformed_values(mapped_point_coordinates, LX, LY, wp_thickness, a2, a3, d)
        print("Next rotate; θ1=", theta1, "θ2=", theta2, "Z=", Z)
        new_theta1, new_theta2,new_Z=theta1-prev_theta1, -(theta2-prev_theta2),Z-prev_Z
        prev_theta1,prev_theta2,prev_Z=theta1, theta2,Z
        print("Next relative rotate; θ1_Rel=", new_theta1, "θ2_Rel=", new_theta2, "Z_Rel=", new_Z)

        transformed_shortest_path.append((new_theta1, new_theta2, new_Z))

    print("Shortest path to visit: ", transformed_shortest_path)
    return  transformed_shortest_path


#exec_get_traverse_path('head', 20)
