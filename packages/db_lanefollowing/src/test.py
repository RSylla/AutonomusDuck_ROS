self.arr = SMBus(1)


def arr_input(self):
    array_value = bin(self.arr.read_byte_data(0x3e, 0x11))[2:].zfill(8)
    new_values_list = [-5, -3, -2, -1, 1, 2, 3, 5]
    bitsum = 0
    counter = 0

    for index in range(len(array_value)):
        if array_value[index] == "1":
            bitsum += new_values_list[index]
            counter += 1

    if counter == 0 or counter == 8:
        raise ValueError
    elif 3 <= counter <= 5:
        result = bitsum
    else:
        result = bitsum / counter

    return result

start_time = time.time()
previous_error = 0
integral = 0

# Set initial parameters for duck's devel run (run, Kp, Ki, Kd, v_max).
rospy.set_param("/rpidv", [0, 0.065, 0.000215, 0.01385, 0.42])
"""
rosparam set /rpidv "[1, 0.065, 0.000215, 0.01385, 0.42]"
Best settings so far:
"[1, 0.065, 0.000215, 0.01385, 0.42]"

last: 
"[1, 0.063, 0.000015, 0.0138, 0.45]"
"""
while True:
    # Measure elapsed time
    delta_time = time.time() - start_time

    # Get parameters from ROS
    run, kp, ki, kd, v_max = rospy.get_param("/rpidv")

    # Input is a value from -4 to 4
    try:
        arr_position = node.arr_input()
    except:
        arr_position = previous_error

    # ---- Section for PID controller --------------------------------
    # Since target is 0, the error is actual (arr position).
    target = 0
    error = arr_position - target

    # Proportional:
    p = kp * error
    # print(f"Error: {error}\n"
    #       f"P: {p}")
    # Integral:
    integral += error * delta_time
    i = ki * integral

    # Clamp integral to avoid wind-up
    i = min(max(i, -1), 1)
    if error == 0:
        i = 0
    # print(f"I: {i}")
    # Derivative:
    d = kd * ((error - previous_error) / delta_time)

    pid = min(max(p + i + d, -0.7), 0.7)
    # print(f"D: {d}\n"
    #       f"pid: {pid}")

    # print(f"PID: {pid}")
    # ---- End section ------------------------------------------------

    if run:
        if node.is_obstacle():
            node.shutdown()
        else:
            node.run(v_max, pid)
    else:
        node.shutdown()

    # Overwrite previous error with current error.
    previous_error = arr_position
    start_time = time.time()

    time.sleep(0.02)  # to set loops per sec


def talker():

    pub = rospy.Publisher('/blubot/odometry', String, queue_size=10)
    rospy.init_node('odometry', anonymous=True)
    rate = rospy.Rate(20)  # 10hz
    while not rospy.is_shutdown():

        rate.sleep()