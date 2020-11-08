clear all;

pipe = realsense.pipeline();

cfg = realsense.config;
cfg.enable_stream(realsense.stream.pose, realsense.format.six_dof);
cfg.enable_stream(realsense.stream.fisheye, 1, realsense.format.y8);
cfg.enable_stream(realsense.stream.fisheye, 2, realsense.format.y8);
cfg.enable_stream(realsense.stream.accel, realsense.format.motion_xyz32f);
cfg.enable_stream(realsense.stream.gyro, realsense.format.motion_xyz32f);

pipe.start(cfg);

pipeline_profiles = pipe.get_active_profile();
left_camera = pipeline_profiles.get_stream(realsense.stream.fisheye, 1).as('video_stream_profile');
right_camera = pipeline_profiles.get_stream(realsense.stream.fisheye, 2).as('video_stream_profile');
pose_profile = pipeline_profiles.get_stream(realsense.stream.pose).as('stream_profile');
left_intrinsics = left_camera.get_intrinsics();
right_intrinsics = right_camera.get_intrinsics();

% Extract pose to camera extrinsincs
pose_to_leftcam_extrinsics = pose_profile.get_extrinsics_to(left_camera);
LEFT_T_P = [reshape(pose_to_leftcam_extrinsics.rotation, [3,3]), pose_to_leftcam_extrinsics.translation'; 0,0,0,1];
pose_to_rightcam_extrinsics = pose_profile.get_extrinsics_to(right_camera);
RIGHT_T_P = [reshape(pose_to_rightcam_extrinsics.rotation, [3,3]), pose_to_rightcam_extrinsics.translation'; 0,0,0,1];

tic;

tf = {};
while (toc < 3)
    frames = pipe.wait_for_frames();
    pose_frame = frames.get_pose_frame().as('pose_frame');
    pose_data = pose_frame.get_pose_data();
    tf{end+1} = [quat2rotm([pose_data.rotation(4), pose_data.rotation(1:3)]), pose_data.translation'; 0,0,0,1];
    pause(0.1);
end

pipe.stop();


%%
% Frames:
%   W = World frame, z-axis up
%   M = Map frame (internal to T265), y-axis up
%   P = Pose frame
%   LEFT = Left camera frame
%   RIGHT = Right camera frame
%M_T_P = [quat2rotm([pose_data.rotation(4), pose_data.rotation(1:3)]), pose_data.translation'; 0,0,0,1];

W_T_M = [rotx(pi/2), zeros(3,1); 0,0,0,1];
Pp_T_P = [rotx(pi/2), zeros(3,1); 0,0,0,1];

figure(1);
clf;
%set(gcf,'Color','k');
set(gca,'Color','k');
%set(gca,'XColor',color,'YColor',color,'ZColor',color,'TickDir','out')

for (i = 1:length(tf))
    M_T_P = tf{i};
    
    W_T_Pp = W_T_M * M_T_P * inv(Pp_T_P);
    W_T_LEFT = W_T_M * M_T_P * inv(LEFT_T_P);
    W_T_RIGHT = W_T_M * M_T_P * inv(RIGHT_T_P);

    hold on;
    plotTransform(W_T_Pp);
    plotTransform(W_T_LEFT, 'Left');
    plotTransform(W_T_RIGHT, 'Right');
    hold off;
end

grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');