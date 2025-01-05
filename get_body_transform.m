function struct_active = get_body_transform(struct_active, dt)
    %GET_BODY_TRANSFORM Calculate relative transformation and covariances
    %   This function computes the changes in position and orientation from the 
    %   old pose to the current pose, and transforms the covariances into the
    %   old body frame.


    if exist('dt', 'var')
        dt = struct_active.Pose.Time -struct_active.oldPose.Time;

        Fv = dt^2*eye(3);
        Fa = dt^4/4*eye(3);
        Fqv = dt^2;
       
        oldVelCov = struct_active.oldPose.Vel_Cov; % Current position covariance matrix
        oldAccCov = struct_active.oldPose.Acc_Cov; % Current position covariance matrix
        old_angvel_Cov = struct_active.oldPose.angvel_Cov;
    else

        Fv = zeros(3);
        Fa = zeros(3);
        Fqv = 0;

        oldVelCov = zeros(3); % Current position covariance matrix
        oldAccCov = zeros(3); % Current position covariance matrix
        old_angvel_Cov = 0;
    end

    % Extract current and old poses
    currentPoint = struct_active.Pose.Point.';  % Transpose to match required vector orientation
    currentQuat = struct_active.Pose.Quat;      % Current orientation as a quaternion
    currentPointCov = struct_active.Pose.Point_Cov; % Current position covariance matrix
    oldPointCov = struct_active.oldPose.Point_Cov; % Current position covariance matrix

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
    R = quat2rotm(oldQuat);               % Rotation matrix from old orientation
    % Transform the covariance matrix
    if(struct_active.type=="lidar")
        transformedPosCov = currentPointCov - oldPointCov; % Rotate the position covariance matrix into the old body frame
        transformedQuatCov = struct_active.Pose.Quat_Cov - struct_active.oldPose.Quat_Cov - old_angvel_Cov*Fqv; 
    elseif(struct_active.type=="kalman")
        [Q, Lambda] = eig(currentPointCov - oldPointCov);
        Lambda = max(zeros(3),Lambda);
        transformedPosCov = (Q*Lambda*Q' + R' * oldVelCov * R*Fv + R' * oldAccCov* R * Fa + R' * eye(3)*1e-6 *R)/2;% + eye(3)*1e-6; % Rotate the position covariance matrix into the old body frame
        
        [Q, Lambda] = eig(struct_active.Pose.Quat_Cov - struct_active.oldPose.Quat_Cov);
        Lambda = max(zeros(3),Lambda);
        transformedQuatCov = (Q*Lambda*Q' + old_angvel_Cov * Fqv)/2; 
    else
        transformedPosCov = zeros(3); % Rotate the position covariance matrix into the old body frame
        transformedQuatCov = 0; 
    end
    % 4. Transform orientation covariance to old body frame
    % Assuming orientation covariance is scalar and ratio reflects the relative uncertainty

    % Update the struct with the new calculated values
    struct_active.Twist.Point = deltaPos.';     % Update struct with transformed position change
    struct_active.Twist.Quat = deltaQuat;       % Update struct with calculated orientation change
    if(struct_active.type ~= "uncorrected")
        struct_active.Twist.Point_Cov = transformedPosCov; % Update struct with transformed position covariance
        struct_active.Twist.Quat_Cov = transformedQuatCov(1); % Update struct with transformed orientation covariance
    end
end
