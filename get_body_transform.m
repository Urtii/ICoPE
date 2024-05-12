function struct_active = get_body_transform(struct_active)
    %GET_BODY_TRANSFORM Calculate relative transformation and covariances
    %   This function computes the changes in position and orientation from the 
    %   old pose to the current pose, and transforms the covariances into the
    %   old body frame.

    % Extract current and old poses
    currentPoint = struct_active.Pose.Point.';  % Transpose to match required vector orientation
    currentQuat = struct_active.Pose.Quat;      % Current orientation as a quaternion
    currentPointCov = struct_active.Pose.Point_Cov; % Current position covariance matrix

    oldPoint = struct_active.oldPose.Point.';   % Transpose to match required vector orientation
    oldQuat = struct_active.oldPose.Quat;       % Previous orientation as a quaternion

    % 1. Calculate position change in old body frame
    % Compute the difference in the world frame
    deltaPos_world = currentPoint - oldPoint;   % Vector difference of position in world frame
    % Transform to old body frame using the conjugate of the old quaternion
    deltaPos = rotatepoint(conj(oldQuat), deltaPos_world); % Apply rotation to change frame

    % 2. Calculate orientation change in old body frame
    % Compute relative quaternion
    deltaQuat = conj(oldQuat) * currentQuat;    % Quaternion representing rotation from old to current orientation

    % 3. Transform position covariance to old body frame
    % Create a rotation matrix from the conjugate of the old quaternion
    R = quat2rotm(conj(oldQuat));               % Rotation matrix from old orientation
    % Transform the covariance matrix
    transformedPosCov = R * currentPointCov * R'; % Rotate the position covariance matrix into the old body frame

    % 4. Transform orientation covariance to old body frame
    % Assuming orientation covariance is scalar and ratio reflects the relative uncertainty
    transformedQuatCov = struct_active.Pose.Quat_Cov / struct_active.oldPose.Quat_Cov; 

    % Update the struct with the new calculated values
    struct_active.Twist.Point = deltaPos.';     % Update struct with transformed position change
    struct_active.Twist.Point_Cov = transformedPosCov; % Update struct with transformed position covariance
    struct_active.Twist.Quat = deltaQuat;       % Update struct with calculated orientation change
    struct_active.Twist.Quat_Cov = transformedQuatCov; % Update struct with transformed orientation covariance
end
