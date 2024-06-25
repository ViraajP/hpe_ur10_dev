import rtde_control
import rtde_receive

def LotusCut1(rtde_c):
    # Connect to the RTDE interface
    rtde_c.connect()

    # Add MODBUS signals and set update frequencies
    modbus_signals = [
        (5000, "MODBUS_1"),
        (5001, "MODBUS_2"),
        (5002, "MODBUS_3"),
        (5003, "MODBUS_4"),
        (5004, "MODBUS_5"),
        (5005, "MODBUS_6"),
        (5006, "MODBUS_7"),
        (5007, "MODBUS_8"),
        (5008, "MODBUS_9"),
        (6000, "MODBUS_10"),
        (6001, "MODBUS_11"),
        (6002, "MODBUS_12"),
        (6003, "MODBUS_13"),
        (6004, "MODBUS_14"),
        (6005, "MODBUS_15"),
        (6006, "MODBUS_16"),
        (6007, "MODBUS_17"),
        (6008, "MODBUS_18"),
        (6009, "MODBUS_19"),
        (6010, "MODBUS_20"),
        (6011, "MODBUS_21"),
        (6012, "MODBUS_22")
    ]
    for address, name in modbus_signals:
        rtde_c.add_modbus_signal(address, 2, name)
        rtde_c.set_modbus_signal_update_frequency(name, 10)

    # Set runstate dependent choices
    for i in range(10, 23):
        rtde_c.set_modbus_runstate_dependent_choice(f"MODBUS_{i}", 0)

    # Set gravity, safety mode transition hardness, analog input domains, etc.
    rtde_c.set_gravity([0.0, 0.0, 9.82])
    rtde_c.set_safety_mode_transition_hardness(1)
    rtde_c.set_standard_analog_input_domain(0, 1)
    rtde_c.set_standard_analog_input_domain(1, 1)
    rtde_c.set_tool_analog_input_domain(0, 1)
    rtde_c.set_tool_analog_input_domain(1, 1)
    rtde_c.set_analog_outputdomain(0, 0)
    rtde_c.set_analog_outputdomain(1, 0)
    rtde_c.set_input_actions_to_default()
    rtde_c.set_tool_communication(False, 115200, 0, 1, 1.5, 3.5)
    rtde_c.set_tool_output_mode(0)
    rtde_c.set_tool_digital_output_mode(0, 1)
    rtde_c.set_tool_digital_output_mode(1, 1)
    rtde_c.set_tool_voltage(0)
    rtde_c.set_tcp((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    rtde_c.set_target_payload(1.5, [-0.003, -0.016, 0.015], [0.003026, 0.003026, 0.003026, 0.0, 0.0, 0.0])

    # Run the step counter thread
    rtde_c.run_step_counter_thread()

    # Define Waypoint positions and orientations
    Waypoint_1_p = [0.324583929523971, 0.28459710089825696, 0.2063041100669342, -1.5504866414703118, -0.11856045101363002, -0.09277982230916273]
    Waypoint_1_q = [0.10710207372903824, -1.806059022943014, -2.6484761238098145, -1.8061233959593714, -1.5324557463275355, -3.255179230366842]
    Waypoint_3_p = [0.424061181388, -0.348510104244, 0.106936502288, -1.305559757770, 1.080362740484, -1.253223171236]
    Waypoint_3_q = [-0.47821361223329717, -1.935845514337057, -2.3895022869110107, -1.9287215671935023, -2.1177499930011194, -3.2393410841571253]

    # Main program loop
    while True:
        # Move to Waypoint_1
        rtde_c.move_j(Waypoint_1_p, qnear=Waypoint_1_q, acc=1.3962634015954636, vel=1.0471975511965976)

        # Apply force and move in the specified direction
        rtde_c.zero_ftsensor()
        rtde_c.force_mode(tool_pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), selection_vector=[0, 0, 1, 0, 0, 0], wrench=[0.0, 0.0, -2.0, 0.0, 0.0, 0.0], type=2, limits=[0.1, 0.1, 0.15, 0.3490658503988659, 0.3490658503988659, 0.3490658503988659])

        # Move towards a specified direction
        towardsPos = calculate_point_to_move_towards((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (0.0, 0.0, -1.0), 1000.0)
        rtde_c.move_l(towardsPos, acc=1.2, vel=0.02)

        # Retract in case of tool contact
        stepsToRetract = tool_contact(direction=targetTcpDirection)
        if stepsToRetract > 0:
            rtde_c.stopl(3.0)
            backTrackMovement = get_actual_joint_positions_history(stepsToRetract)
            contactPose = get_forward_kin(backTrackMovement)
            posDir = (targetTcpDirection[0], targetTcpDirection[1], targetTcpDirection[2])
            retractTo = contactPose
            if norm(posDir) > 1e-6:
                normalizedPosDir = normalize(posDir)
                additionalRetraction = (normalizedPosDir[0] * 0.0, normalizedPosDir[1] * 0.0, normalizedPosDir[2] * 0.0, 0, 0, 0)
                retractTo = pose_sub(contactPose, additionalRetraction)
            rtde_c.move_l(retractTo, acc=3.0, vel=0.1)

        # Repeat the process for 5 times
        for _ in range(5):
            rtde_c.zero_ftsensor()
            rtde_c.force_mode(tool_pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), selection_vector=[0, 0, 1, 0, 0, 0], wrench=[0.0, 0.0, -2.0, 0.0, 0.0, 0.0], type=2, limits=[0.1, 0.1, 0.15, 0.3490658503988659, 0.3490658503988659, 0.3490658503988659])
            rtde_c.move_l(Waypoint_1_p, acc=1.2, vel=0.25)
            rtde_c.move_l(Waypoint_3_p, acc=1.2, vel=0.25)

        # Halt the program
        rtde_c.halt()

# Example usage:
if __name__ == "__main__":
    rtde_c = rtde_control.RTDEControlInterface("192.168.0.1")
    LotusCut1(rtde_c)
