function transform = tf2_to_transform(tf2)
    trans = zeros(3,1);
    quat = zeros(4,1);
    
    trans(1) = tf2.Transform.Translation.X;
    trans(2) = tf2.Transform.Translation.Y;
    trans(3) = tf2.Transform.Translation.Z;
    quat(1) = tf2.Transform.Rotation.W;
    quat(2) = tf2.Transform.Rotation.X;
    quat(3) = tf2.Transform.Rotation.Y;
    quat(4) = tf2.Transform.Rotation.Z;
    
    transform = posRotm2tform(trans, quat2rotm(quat));
end