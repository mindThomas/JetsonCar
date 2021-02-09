function tf2 = get_realsense_tf2(bag, time)
    if (~isa(time, 'ros.msg.Time'))
        time = rostime(time);
    end

    to = 'realsense_t265_odom_frame';
    from = 'realsense_t265_pose_frame';
    tf2 = get_tf2(bag, time, from, to);
end