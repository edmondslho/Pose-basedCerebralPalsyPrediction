clear;
close all;
load('B_25J_MRGB_correctedPoseData.mat');

vidRange = 1:12;
jointRange = 1:21;
frmRange = 0:998;


%% Calculating the rotation to align imported spine alignment with perfect Y

for vid=vidRange
    allTheta{vid} = {};
    
    for frm=frmRange

            oldPose = [allAlignmentVec{vid}{frm+1}];
            vec = [ oldPose ; 0 0];

            yAxis = [ 0 1 ];
            perfectY = [ yAxis ; 0 0];

            theta = acosd( dot(oldPose, yAxis) / norm(oldPose) * norm(yAxis) );
            val = cross([oldPose,0], [yAxis,0]);
            sign = 1;
            if (sum(val)<0)
                sign = -1;
            end
           
            rM = getRotZ(sign*theta);

            vec = [ (rM*oldPose')' ; 0 0];            

            allFinalPose {vid} {frm+1} = [ (rM*correctedPose {vid} {frm+1}(:,:)')' ];

            allTheta {vid} {frm+1} = theta;

    end
end

%% pID - defining the Parent-Child joint relationships 

% joint ID
% 1 - nose
% 2 - neck
% 3 - rShoulder
% 4 - rElbow
% 5 - rWrist
% 6 - lShoulder
% 7 - lElbow
% 8 - lWrist
% 9 - midHip
% 10 - rHip
% 11 - rKnee
% 12 - rFoot
% 13 - lHip
% 14 - lKnee
% 15 - lFoot
% 16 - rEye (currently unused, removed from pID)
% 17 - lEye (currently unused, removed from pID)
% 18 - rEar (currently unused, removed from pID)
% 19 - lEar (currently unused, removed from pID)
% 20 (16) - LBigToe 
% 21 (17) - LSmallToe
% 22 (18) - LHeel
% 23 (19) - RBigToe
% 24 (20) - RSmallToe
% 25 (21) - RHeel
% 26 - Background (currently unused, removed from pID)

% deinfe the parent-child relationship between joints
pID = [ 2 9 2 3 4 2 6 7 0 9 10 11 9 13 14 15 15 15 12 12 12 ];

%% Exporting the Pre-processed Data

save('C_25J_MRGB_ProcessedData.mat', 'allFinalPose', 'pID');

%% Functions

function rM = getRotZ(theta)
rM = [ cosd(theta)   -sind(theta) ; sind(theta)   cosd(theta) ];
end

