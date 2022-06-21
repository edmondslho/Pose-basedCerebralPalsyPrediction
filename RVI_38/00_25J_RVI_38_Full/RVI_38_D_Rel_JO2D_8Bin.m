%% Initialization
clear;
close all;

load('C_25J_RVI_38_ProcessedData.mat');
vidRange = 1:38;
numBin = 8;
theta = (2*pi)/numBin; 
vec = [0,1]';
jFullBodyRoot = 1:21;

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

%% for all videos

H_jFullBody8RelOri = {};

allJoints = [1:15];


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
    
    H_jFullBody8RelOri{vid} = fusedHist;

end  
    

%% Reshaping for remaining pipeline

Rel_JO_Parts8 = {};
Rel_JO_Parts8  = reshape(cell2mat(H_jFullBody8RelOri), [], (numel(vidRange)))';


%%
HORJO2D_LW8 = Rel_JO_Parts8(:,57:64);
HORJO2D_LE8 = Rel_JO_Parts8(:,49:56);
HORJO2D_RW8 = Rel_JO_Parts8(:,33:40);
HORJO2D_RE8 = Rel_JO_Parts8(:,25:32);
HORJO2D_LA8 = Rel_JO_Parts8(:,113:120);
HORJO2D_LK8 = Rel_JO_Parts8(:,105:112);
HORJO2D_RA8 = Rel_JO_Parts8(:,89:96);
HORJO2D_RK8 = Rel_JO_Parts8(:,81:88);

HORJO2D_Larm8 = [HORJO2D_LW8+HORJO2D_LE8];
HORJO2D_Lleg8 = [HORJO2D_RW8+HORJO2D_RE8];
HORJO2D_Rarm8 = [HORJO2D_LA8+HORJO2D_LK8];
HORJO2D_Rleg8 = [HORJO2D_RA8+HORJO2D_RK8];


%% Export all Histogram data (HOJO2D) for remaining pipeline
save(['25J_RVI_38_Rel_JO_' num2str(numBin) 'bins' '.mat'], 'Rel_JO_Parts8','HORJO2D_Larm8','HORJO2D_Lleg8', 'HORJO2D_Rarm8', 'HORJO2D_Rleg8' );

%% Functions
function [rM] = getRotationMat(angle)
    rM = [ cos(angle)   -sin(angle) ; sin(angle)  cos(angle) ];
end