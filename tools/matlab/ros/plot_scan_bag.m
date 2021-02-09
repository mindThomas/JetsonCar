function plot_scan_bag(scan, bag, varargin)
    if (length(varargin))
        realsense_T_lidar = varargin{1};
    else
        realsense_T_lidar = eye(4);
    end

    tf2 = get_tf2(bag, stamp_to_seconds(scan.Header.Stamp), 'realsense_t265_pose_frame', 'realsense_t265_odom_frame'); % scan.Header.FrameId, world
    odom_T_realsense = tf2_to_transform(tf2);   
    odom_T_lidar = odom_T_realsense * realsense_T_lidar;
    
    plot_scan(scan, odom_T_lidar);   
end