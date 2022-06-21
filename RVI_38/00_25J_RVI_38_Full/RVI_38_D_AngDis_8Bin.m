%% Initialization
clear;
close all;

load('C_25J_RVI_38_ProcessedData.mat');

displacement_interval = 10;

vidRange = 1:38;

numBin = 8;
maxDegree = 180;
AngDisBins = zeros(1,numBin);
AngDisBins(numBin) = maxDegree;
for i = numBin-1:-1:1
    AngDisBins(i) = maxDegree/2;
    maxDegree = AngDisBins(i);
end

theta = (2*pi)/numBin;
vec = [0,1]';

jRightArm = 4:5;
jLeftArm = 7:8;
jRightLeg = 11:12; 
jLeftLeg = 14:15;
jFullBody = [1:8 10:21];


H_allJoints8 = {};
H_jRightElbow8 = {};
H_jRightWrist8 = {};
H_jLeftElbow8 = {};
H_jLeftWrist8 = {};
H_jRightKnee8 = {};
H_jRightAnkle8 = {};
H_jLeftKnee8 = {};
H_jLeftAnkle8 = {};

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


%% Angular Displacement

for vid = vidRange
    
    for allJoints = jFullBody
        
        frmRange = 1+displacement_interval:displacement_interval:length(allFinalPose{vid}(1,:))-1;
        frmLength = length(frmRange);
                
        curHist = zeros(1,numBin);
        
        for frm = frmRange
            for jID = [allJoints]
                bone = allFinalPose{vid}{frm}(jID,1:2) - allFinalPose{vid}{frm}(pID(jID),1:2);    
                oldbone = allFinalPose{vid}{frm-displacement_interval}(jID,1:2) - allFinalPose{vid}{frm-displacement_interval}(pID(jID),1:2);
                
                if norm(bone)>0 && norm(oldbone)>0
                    bone = bone/norm(bone);
                    oldbone = oldbone/norm(oldbone);
                    u = bone;
                    v = oldbone;
                    
                    
                    CosTheta = dot(u,v)/(norm(u)*norm(v));
                    ThetaInDegrees = acosd(CosTheta);
                    
                    for aa=1:numBin
                        if (ThetaInDegrees < AngDisBins(aa))
                            bestBin = aa;
                            break; 
                        end
                    end

                    curHist(bestBin) = curHist(bestBin) + 1;
                end
            end
        end
        

        normHist = curHist/sum(curHist);        
        H_allJoints8{vid}{allJoints} = normHist;
    end
    
H_jRightElbow8 {vid} = [H_allJoints8{vid}{min(jRightArm)}];
H_jRightWrist8 {vid} = [H_allJoints8{vid}{max(jRightArm)}];

H_jLeftElbow8 {vid} = [H_allJoints8{vid}{min(jLeftArm)}];
H_jLeftWrist8 {vid} = [H_allJoints8{vid}{max(jLeftArm)}];

H_jRightKnee8 {vid} = [H_allJoints8{vid}{min(jRightLeg)}];
H_jRightAnkle8 {vid} = [H_allJoints8{vid}{max(jRightLeg)}];

H_jLeftKnee8 {vid} = [H_allJoints8{vid}{min(jLeftLeg)}];
H_jLeftAnkle8 {vid} = [H_allJoints8{vid}{max(jLeftLeg)}];   
    
end

%% Reshaping for remaining pipeline

AngDisp_RightElbow8 = {};
AngDisp_RightElbow8  = reshape(cell2mat(H_jRightElbow8), (numBin), (numel(vidRange)))';

AngDisp_RightWrist8 = {};
AngDisp_RightWrist8  = reshape(cell2mat(H_jRightWrist8), (numBin), (numel(vidRange)))';

AngDisp_RightKnee8 = {};
AngDisp_RightKnee8  = reshape(cell2mat(H_jRightKnee8), (numBin), (numel(vidRange)))';

AngDisp_RightAnkle8 = {};
AngDisp_RightAnkle8  = reshape(cell2mat(H_jRightAnkle8), (numBin), (numel(vidRange)))';

AngDisp_LeftElbow8 = {};
AngDisp_LeftElbow8  = reshape(cell2mat(H_jLeftElbow8), (numBin), (numel(vidRange)))';

AngDisp_LeftWrist8 = {};
AngDisp_LeftWrist8  = reshape(cell2mat(H_jLeftWrist8), (numBin), (numel(vidRange)))';

AngDisp_LeftKnee8 = {};
AngDisp_LeftKnee8  = reshape(cell2mat(H_jLeftKnee8), (numBin), (numel(vidRange)))';

AngDisp_LeftAnkle8 = {};
AngDisp_LeftAnkle8  = reshape(cell2mat(H_jLeftAnkle8), (numBin), (numel(vidRange)))';


%% Export all Histogram data (HOJD2D) for remaining pipeline
save(['25J_RVI_38_AngDisp_' num2str(numBin) 'bins' '.mat'], 'AngDisp_RightElbow8', 'AngDisp_RightWrist8', 'AngDisp_RightKnee8', 'AngDisp_RightAnkle8', 'AngDisp_LeftElbow8', 'AngDisp_LeftWrist8', 'AngDisp_LeftKnee8', 'AngDisp_LeftAnkle8');


%% Functions
function [rM] = getRotationMat(angle)
    rM = [ cos(angle)   -sin(angle) ; sin(angle)  cos(angle) ];
end