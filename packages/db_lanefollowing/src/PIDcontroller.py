def error_calculator(array_value):
    new_values_list = [-6, -4, -2, -0.5, 0.5, 2, 4, 6]
    bitsum = 0
    counter = 0
    left_turn = False
    left_turn_array_value_list = ["101", "1001", "10001"]  # ['10010000', '11001000', '10011000', '11001100',
    # '01000100','00100100', '01001000', '10001000','01001100']
    if array_value in left_turn_array_value_list:
        left_turn = True

    for index in range(len(array_value)):
        if array_value[index] == "1":
            bitsum += new_values_list[index]
            counter += 1
    if counter == 0 or counter == 8:
        raise ValueError
    elif 3 <= counter <= 6:
        error = bitsum
    else:
        error = bitsum / counter
    return error, left_turn


def pid_controller(error, integral, prev_error, delta_time, kp, ki, kd):
    # Since target is 0, the error is actual (arr position).
    # Proportional:
    p = kp * error
    # Integral:
    integral += error * delta_time
    i = ki * integral
    # Clamp integral to avoid wind-up
    i = min(max(i, -1), 1)
    if error == 0:
        i = 0
    # Derivative:
    d = kd * ((error - prev_error) / delta_time)
    pid = min(max(p + i + d, -0.4), 0.4)
    return pid, i, error
