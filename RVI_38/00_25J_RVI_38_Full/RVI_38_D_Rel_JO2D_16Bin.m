%% Initialization
clear;
close all;
load('C_25J_RVI_38_ProcessedData.mat');

vidRange = 1:38;
numBin = 16;
theta = (2*pi)/numBin;
vec = [0,1]'; 
jFullBodyRoot = 1:21;
allJoints = [1:15];

for i=1:numBin
    rM = getRotationMat(i*theta);
    newVec = rM*vec;
end

binVec = zeros(2,numBin);
theta = (2*pi)/numBin;
vec = [0,1]';
for i=1:numBin
    rM = getRotationMat(i*theta);
    binVec(:,i) = rM*vec;
end

H_jFullBody16RelOri = {};

%% Full Body Relative Orientation

for vid = vidRange
    
    
    frmLength = length(allFinalPose{vid}(1,:))-1;
    frmRange = 1:frmLength;
    curHist = {};
    for i=1:length(allJoints)
        curHist{i} = zeros(1,numBin);
    end
    
    for frm = frmRange
       
        for aa = 1:length(allJoints)     
            jID = jFullBodyRoot(allJoints(aa));
            for bb = 1:length(allJoints)
                jID2 = jFullBodyRoot(allJoints(bb));
             
                bone = allFinalPose{vid}{frm}(jID,1:2) - allFinalPose{vid}{frm}(jID2,1:2);

                if norm(bone)>0
                    bone = bone/norm(bone);
                    curHist{aa} = curHist{aa} + getHist(bone, binVec, true);
                end
            end
        end
    end   
        
  
    fusedHist = [];
    for i=1:length(allJoints)
        if sum(curHist{i})>0
            curHist{i} = curHist{i}/sum(curHist{i});
        end
        fusedHist = [ fusedHist curHist{i} ];
    end
    
    H_jFullBody16RelOri{vid} = fusedHist;
    
end  


%% Reshaping for remaining pipeline

Rel_JO_Parts16 = {};
Rel_JO_Parts16 = reshape(cell2mat(H_jFullBody16RelOri), [], (numel(vidRange)))';

HORJO2D_LW16 = Rel_JO_Parts16(:,113:128);
HORJO2D_LE16 = Rel_JO_Parts16(:,97:112);
HORJO2D_RW16 = Rel_JO_Parts16(:,65:80);
HORJO2D_RE16 = Rel_JO_Parts16(:,49:64);
HORJO2D_LA16 = Rel_JO_Parts16(:,225:240);
HORJO2D_LK16 = Rel_JO_Parts16(:,209:224);
HORJO2D_RA16 = Rel_JO_Parts16(:,177:192);
HORJO2D_RK16 = Rel_JO_Parts16(:,161:176);

HORJO2D_Larm16 = [HORJO2D_LW16+HORJO2D_LE16];
HORJO2D_Lleg16 = [HORJO2D_LA16+HORJO2D_LK16];
HORJO2D_Rarm16 = [HORJO2D_RW16+HORJO2D_RE16];
HORJO2D_Rleg16 = [HORJO2D_RA16+HORJO2D_RK16];


%% Export all Histogram data (HOJO2D) for remaining pipeline
save(['25J_RVI_38_Rel_JO_' num2str(numBin) 'bins' '.mat'], 'Rel_JO_Parts16','HORJO2D_Larm16','HORJO2D_Lleg16', 'HORJO2D_Rarm16', 'HORJO2D_Rleg16' );


%% Functions
function [rM] = getRotationMat(angle)
    rM = [ cos(angle)   -sin(angle) ; sin(angle)  cos(angle) ];
end