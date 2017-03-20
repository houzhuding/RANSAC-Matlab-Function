function [I,extreme_pts] = ransac_ding(I,pts,iter_num,dist_thres,inlier_num,line_num )
%% RANSAC Use Random Sample Consensus to fit  lines
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% I          =  original image (gray scale)
%%%%% pts        =  points found after non max suppression
%%%%% iter_num   =  iteration time
%%%%% dist_thres =  distance threshold
%%%%% inlier_num =  inlier number threshold for certify the fitted line
%%%%% line_num   =  line number needed to find
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Problem RANSAC line fitting...\n\n');
pts_num     = 2; % number of points for fitting
line_found  = 0; % initial found line is 0
extreme_pts = zeros(2*line_num,2); % store the extreme two points of all lines
it          = 0; % iteration number     
inlier_numN = 0; % initial a new inlier number for sake of current setting is two big
inlier_num_ori = inlier_num; % store the original inlier numer;

% Convert gray image to rgb in order to circle inliers in red
if ~(size(I,3)==3)
   I = cat(3,I,I,I); 
end
while line_found < line_num
    % define the vector to store inliers index all random points
    pts_selected = []; 
    % define the vector to store inliers every iteration
    inliers = []; 
    % generate two random indexs
    pts_order = randi([1 size(pts,2)],1,pts_num); 
    % select to random points by random indexs
    pts_sel = pts(:,pts_order); 
    pt1 = pts_sel(:,1);pt2 = pts_sel(:,2);
    % calculate the distance of rest of the points to line
    for j = 1:size(pts,2)
        dist = point_to_line_ding(pt1,pt2,pts(:,j));
        % save inliers 
        if dist < dist_thres
            pts_selected = [pts_selected,j];
            inliers =[inliers,pts(:,j)];
        end
    end
    % if number of inliers exceed certain number, then one line found
    if size(inliers,2) > inlier_num
        line_found = line_found + 1;
        % circle every inlier by a red 3 by 3 square box
        for cnt = 1:size(inliers,2) 
            draw_square_ding(inliers(1,cnt),inliers(2,cnt));
        end
        % find the two extreme x coordiants
        x_min = min(inliers(1,:));
        x_max = max(inliers(1,:));
        
        % find the two extreme inliers
        p1 = find(inliers(1,:) == x_min,1,'first');
        p2 = find(inliers(1,:) == x_max,1,'last');
        
        % store the two extreme inliers
        extreme_pt = [inliers(:,p1)';inliers(:,p2)'];
        extreme_pts(2*line_found-1:2*line_found,:) = extreme_pt;
        
        % remove the qualified inliers 
        pts(:,pts_selected) = [] ; % remove the inliers 
        fprintf('Line %d found after %d iterations\n\n',line_found,it)
    end
    % stop when all line  found
    if line_found == line_num
        return
    end
    if it > iter_num
        return
    end
    it = it + 1;% count iteration number
    
    % Ensure to find proper line without too much iteration
    % e.g. if after 1e4 iteration still can not find any line 
    % reduce the inlier number by 1...
    if line_found == 0 && it > 1e4 
        inlier_numN = inlier_num_ori - 1;
    end
    if line_found == 1 && it > 2e4 
        inlier_numN = inlier_num_ori - 2;
    end
    if  line_found == 2 && it > 3e4
        inlier_numN = inlier_num_ori - 3;
    end
    if  line_found == 3 && it > 4e4
        inlier_numN = inlier_num_ori - 4;
    end
    % Update the new inlier number if needed
    if inlier_numN
        inlier_num = inlier_numN;
    end
end

%% Function to draw a square to cover the point
function draw_square_ding(x,y)
        % 3x3 square set pixel to red (255,0,0)
        for ii = -1:1
            for jj = -1:1
                if y+ii>-1&&y+ii<size(I,1)+1 && x+jj>-1&&x+jj<size(I,2)+1
                % detection for boundary, if out the img, then
                % use part of the square to cover the point
                    I(y+ii,x+jj,:) = [255,0,0];% Set the circle black.
                end
            end
        end
end

end % End of RANSAC 

