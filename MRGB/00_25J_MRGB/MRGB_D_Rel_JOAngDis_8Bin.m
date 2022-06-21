%% Initialization
clear;
close all;

load('C_25J_MRGB_ProcessedData.mat');
displacement_interval = 8;
vidRange = 1:12;
numBin = 8;
maxDegree = 180;
AngDisBins = zeros(1,numBin);
AngDisBins(numBin) = maxDegree;
for i = numBin-1:-1:1
    AngDisBins(i) = maxDegree/2;
    maxDegree = AngDisBins(i);
end

theta = (2*pi)/numBin;

jRightArm = 4:5;
jLeftArm = 7:8;
jRightLeg = 11:12;
jLeftLeg = 14:15;
jTorso = [1:3 6 9 10 13];
allJoints = [1:15];

vec = [0,1]';

jFullBodyRoot = 1:21;


%% for all videos

H_jFullBody8RelOri = {};


%% Full Body Relative Orientation

for vid = vidRange   
    
    frmLength = length(allFinalPose{vid}(1,:))-1;
    frmRange = 1+displacement_interval:displacement_interval:frmLength;
    
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
                oldbone = allFinalPose{vid}{frm-displacement_interval}(jID,1:2) - allFinalPose{vid}{frm-displacement_interval}(jID2,1:2);

                if norm(bone)>0  
                    bone = bone/norm(bone);
                    oldbone = oldbone/norm(oldbone);
                    u = bone;
                    v = oldbone;
                    
                    
                    CosTheta = dot(u,v)/(norm(u)*norm(v));
                    ThetaInDegrees = acosd(CosTheta);
                    
                    curHist{aa} = curHist{aa} + getHist(ThetaInDegrees, AngDisBins, false);
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

Rel_JO_AngDis_Parts8 = {};
Rel_JO_AngDis_Parts8  = reshape(cell2mat(H_jFullBody8RelOri), [], (numel(vidRange)))';

HORJAD2D_LW8 = Rel_JO_AngDis_Parts8(:,57:64);
HORJAD2D_LE8 = Rel_JO_AngDis_Parts8(:,49:56);
HORJAD2D_RW8 = Rel_JO_AngDis_Parts8(:,33:40);
HORJAD2D_RE8 = Rel_JO_AngDis_Parts8(:,25:32);
HORJAD2D_LA8 = Rel_JO_AngDis_Parts8(:,113:120);
HORJAD2D_LK8 = Rel_JO_AngDis_Parts8(:,105:112);
HORJAD2D_RA8 = Rel_JO_AngDis_Parts8(:,89:96);
HORJAD2D_RK8 = Rel_JO_AngDis_Parts8(:,81:88);

HORJAD2D_Larm8 = [HORJAD2D_LW8+HORJAD2D_LE8];
HORJAD2D_Lleg8 = [HORJAD2D_LA8+HORJAD2D_LK8];
HORJAD2D_Rarm8 = [HORJAD2D_RW8+HORJAD2D_RE8];
HORJAD2D_Rleg8 = [HORJAD2D_RA8+HORJAD2D_RK8];


%% Export all Histogram data (HOJO2D) for remaining pipeline
save(['25J_MRGB_Rel_JO_AngDis_' num2str(numBin) 'bins' '.mat'], 'Rel_JO_AngDis_Parts8', 'HORJAD2D_Larm8', 'HORJAD2D_Lleg8', 'HORJAD2D_Rarm8', 'HORJAD2D_Rleg8');

%% Functions
function [rM] = getRotationMat(angle)
    rM = [ cos(angle)   -sin(angle) ; sin(angle)  cos(angle) ];
end