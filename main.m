%-----------------------------------------------------------------------
% Description: Implementation of the Kalman filter in order to track
%               people in a sequence of frames (video)
% 
% Resources: frames/ directory -> dataset of images taken from real video
%
% Output: kalman_filter.mp4 -> video resulting of the execution of the KF
%-----------------------------------------------------------------------
% Author: Marcos Canales Mayo
%-----------------------------------------------------------------------

% Could be interesting to consider the floor plane instead of image pixels,
% because the X-axis and Y-axis measure variation also depends on whether
% the person is walking away or getting closer

% Kalman Filter
% Time between frames (s)
diff_t = 1/15;
A = [1 0 diff_t 0;
    0 1 0 diff_t;
    0 0 1 0;
    0 0 0 1];
C = [1 0 0 0;
    0 1 0 0];

% Uncertainty matrices
% Estimated speeds (px/s)
est_vel_x = 40;
est_vel_y = 5;
Q = [(est_vel_x*diff_t)^2 0 diff_t 0;
    0 (est_vel_y*diff_t)^2 0 diff_t;
    diff_t 0 (est_vel_x^2) 0;
    0 diff_t 0 (est_vel_y^2)];
% Estimated HOG detector error (px)
est_err_x = 25;
est_err_y = 25;
R = [est_err_x^2 0;
    0 est_err_y^2];

% frames folder indices
first_img_i = 70;
last_img_i = 299;
% Start the algorithm
peopleDetector = vision.PeopleDetector;
for i = first_img_i:last_img_i
    frame = imread(sprintf('frames\\imgrect_%09d_c1.pgm', i));
    [bboxes, scores] = step(peopleDetector, frame);
    if (size(scores) > 0)
        % Annotate detected people
        frame = insertObjectAnnotation(frame, 'rectangle', bboxes, scores);
        figure (1), imshow(frame)
        title('Prediction through Kalman Filter');
        % Apply KF from first people detection on
        first_img_i = i;
        break;
    end
end

% Kalman Filter
% bboxes = (x y width height)
% mu = (x y vel_x vel_y)
mu_t_t = [bboxes(1, 1)+bboxes(1, 3)/2; bboxes(1, 2)+bboxes(1, 4)/2; 0; 0];
% sigma = 0 then first prediction will be Q
sigma_t_t = 0;
for i = first_img_i:last_img_i
    % Prediction
    mu_t_tm1 = A*mu_t_t;
    sigma_t_tm1 = A*sigma_t_t*A' + Q;
    
    % Matching
    frame = imread(sprintf('frames\\imgrect_%09d_c1.pgm', i));
    [bboxes, scores] = step(peopleDetector, frame);
    % If there's at least one person
    if(size(scores, 1) > 0)
        % measure
        y_t = [bboxes(1, 1) + bboxes(1, 3)/2; bboxes(1, 2) + bboxes(1, 4)/2];
        
        % residue
        r_t = y_t - (C*mu_t_tm1);
        % residual covariance
        S_t = C*sigma_t_tm1*C' + R;
        % Kalman gain
        K_t = (sigma_t_tm1*C')*inv(S_t);
        
        % Check whether measure is spurious
        % Valid measure
        if (abs(r_t(1,1)) < sqrt(S_t(1,1)) && abs(r_t(2,1)) < sqrt(S_t(2,2)))
            % Then update
            mu_t_t = mu_t_tm1 + K_t*r_t;
            sigma_t_t = (eye(size(K_t,1)) - K_t*C)*sigma_t_tm1;
        % Invalid measure (spurious data)
        else
            % Then no update is done
            mu_t_t = mu_t_tm1;
            sigma_t_t = sigma_t_tm1;
        end
        
        % draw figure
        frame = insertObjectAnnotation(frame, 'rectangle', bboxes, 'People Detector', 'Color', 'green');
    % There are no people in the image
    else
        mu_t_t = mu_t_tm1;
        sigma_t_t = sigma_t_tm1;
    end
    
    % uncertainty ellipses
    % velocity is not considered to draw the uncertainty ellipses
    % prediction ellipse
    pred_uncertainty_ellipse = [sigma_t_tm1(1,1) sigma_t_tm1(1,2); sigma_t_tm1(2,1) sigma_t_tm1(2,2)];
    nu_pred_uncertainty_ellipse = [mu_t_tm1(1,1); mu_t_tm1(2,1)];
    % update ellipse
    upd_uncertainty_ellipse = [sigma_t_t(1,1) sigma_t_t(1,2); sigma_t_t(2,1) sigma_t_t(2,2)];
    nu_upd_uncertainty_ellipse = [mu_t_t(1,1); mu_t_t(2,1)];
    
    % prediction bbox
    bbox_pred_size = 200;
    bbox_pred_width = bbox_pred_size;
    bbox_pred_height = bbox_pred_size;
    bbox_pred_x = mu_t_tm1(1,1) - bbox_pred_size/2;
    bbox_pred_y = mu_t_tm1(2,1) - bbox_pred_size/2;
    bbox_prediction = [bbox_pred_x bbox_pred_y bbox_pred_width bbox_pred_height];
    % update bbox
    bbox_upd_size = 240;
    bbox_upd_width = bbox_upd_size;
    bbox_upd_height = bbox_upd_size;
    bbox_upd_x = mu_t_t(1,1) - bbox_upd_size/2;
    bbox_upd_y = mu_t_t(2,1) - bbox_upd_size/2;
    bbox_update = [bbox_upd_x bbox_upd_y bbox_upd_width bbox_upd_height];
    
    % draw bboxes
    frame = insertObjectAnnotation(frame, 'rectangle', bbox_prediction, 'Prediction', 'Color', 'yellow');
    frame = insertObjectAnnotation(frame, 'rectangle', bbox_update, 'Update', 'Color', 'blue');
    figure (1), imshow(frame)
    
    % draw uncertainty ellipses
    plotUncertainEllip2D(pred_uncertainty_ellipse, nu_pred_uncertainty_ellipse, 'yellow');
    plotUncertainEllip2D(upd_uncertainty_ellipse, nu_upd_uncertainty_ellipse, 'blue');
    
    pause(0.5);
end