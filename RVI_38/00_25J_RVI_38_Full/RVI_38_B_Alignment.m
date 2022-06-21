clear;

load('A_25J_RVI_38_interpPoseData.mat');

allRoot = {};
allAlignmentVec = {};

vidRange = 1:38;

jointRange = [1:15, 20:25];

for vid=vidRange
    
    fileRef = sprintf('25J_RVI38_Full_Processed/RVI_38_%04i_*_keypoints.json', vid);
    fileA = dir(fileRef);
    frmLength =length(fileA)-1;
            
    frmRange = 0:frmLength;
     
    for frm=frmRange
                
        j2X = iPose {vid}{frm+1} {2}(1,1);
        j2Y = iPose {vid}{frm+1} {2}(1,2);
        j2 = [j2X, j2Y];
       
        j9X = iPose {vid}{frm+1} {9}(1,1);
        j9Y = iPose {vid}{frm+1} {9}(1,2);
        j9 = [j9X, j9Y];
               
        AlignmentVec = j2 - j9;
        
        allAlignmentVec {vid} {frm+1} = AlignmentVec;
        
        allRoot {vid} {frm+1} = j9;
        
     
    end
end


%% Reshaping the interpolated data
iPoseRe = iPose;

for vid=vidRange
    
    fileRef = sprintf('25J_RVI38_Full_Processed/RVI_38_%04i_*_keypoints.json', vid);
    fileA = dir(fileRef);
    frmLength =length(fileA)-1;
    
    frmRange = 0:frmLength;
    
    for j=frmRange
        tmp = iPoseRe{vid}{j+1};
        iPoseRe{vid}{j+1} = reshape(cell2mat(tmp), 3, length(jointRange))';
    end
end


%% Correcting joint positions for every frame's Root position

for vid=vidRange
    
    fileRef = sprintf('25J_RVI38_Full_Processed/RVI_38_%04i_*_keypoints.json', vid);
    fileA = dir(fileRef);
    frmLength =length(fileA)-1;
     
    frmRange = 0:frmLength;
    
    for frm=frmRange
        for jID = jointRange
            correctedJoints {vid} {frm+1} {jID} = iPose{vid}{frm+1}{jID}(1,1:2)-(allRoot {vid} {frm+1});
        end
    end    
end

%% Reshaping to 2x21 for remaining pipeline

correctedPose = correctedJoints;

for vid=vidRange
    
    fileRef = sprintf('25J_RVI38_Full_Processed/RVI_38_%04i_*_keypoints.json', vid);
    fileA = dir(fileRef);
    frmLength =length(fileA)-1;
    
    frmRange = 0:frmLength;
       
    for j=frmRange
        tmp = correctedPose{vid}{j+1};
        correctedPose{vid}{j+1} = reshape(cell2mat(tmp), 2, 21)';
    end
end


%% Exporting the Corrected Pose Data

save('B_25J_RVI_38_correctedPoseData.mat', 'correctedPose', 'allAlignmentVec','allRoot');