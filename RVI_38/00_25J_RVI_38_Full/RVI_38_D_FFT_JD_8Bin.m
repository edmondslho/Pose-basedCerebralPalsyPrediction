clear;
close all;

load('C_25J_RVI_38_ProcessedData.mat');
vidRange = 1:38;
numBin = 8; 
pwr = 2;

jRightArm = 4:5;
jLeftArm = 7:8;
jRightLeg = 11:12;
jLeftLeg = 14:15;
jFullBody = 1:21;

H_allJoints8 = {};
H_jRightElbow8 = {};
H_jRightWrist8 = {};
H_jLeftElbow8 = {};
H_jLeftWrist8 = {};
H_jRightKnee8 = {};
H_jRightAnkle8 = {};
H_jLeftKnee8 = {};
H_jLeftAnkle8 = {};


%% FFTJD

for vid = vidRange
    
    for allJoints = jFullBody
        
        frmLength = length(allFinalPose{vid}(1,:))-1;
        
        FFTJD_curHist = zeros(1,numBin);
        
        for frm = 2:6:frmLength
            for jID = [allJoints]
                vel = allFinalPose{vid}{frm}(jID,1:2) -allFinalPose{vid}{frm-1}(jID,1:2);
                speed (frm) = norm(vel);
                
            end
        end

        
        Fs = 50;
        T = 1/Fs;
        L = length(speed);
        t = (0:L-1)*T;
        
        Y = fft(speed);
        P2 = abs(Y/L);
        P1 = P2(1:L/2+1);
        P1(2:end-1) = 2*P1(2:end-1);
        
        f = Fs*(0:(L/2))/L;
        
        maxF = max(f);
        
        for bb = 1:length(f)


            if f(bb) < maxF*(1^pwr/numBin^pwr)
                bestBin = 1;
            elseif f(bb) < maxF*(2^pwr/numBin^pwr)
                bestBin = 2;
            elseif f(bb) < maxF*(3^pwr/numBin^pwr)
                bestBin = 3;
            elseif f(bb) < maxF*(4^pwr/numBin^pwr)
                bestBin = 4;
            elseif f(bb) < maxF*(5^pwr/numBin^pwr)
                bestBin = 5;
            elseif f(bb) < maxF*(6^pwr/numBin^pwr)
                bestBin = 6;
            elseif f(bb) < maxF*(7^pwr/numBin^pwr)
                bestBin = 7;
            elseif f(bb) <= maxF
                bestBin = 8;
            end            


            FFTJD_curHist(bestBin) = FFTJD_curHist(bestBin) + P1(bb);
        end
        
        FFTJD_normHist = FFTJD_curHist/sum(FFTJD_curHist);              
                
        H_allJoints8{vid}{allJoints} = FFTJD_normHist;
                
    end
    

H_jRightElbow8 {vid} = [H_allJoints8{vid}{min(jRightArm)}];
H_jRightWrist8 {vid} = [H_allJoints8{vid}{max(jRightArm)}];

H_jLeftElbow8 {vid} = [H_allJoints8{vid}{min(jLeftArm)}];
H_jLeftWrist8 {vid} = [H_allJoints8{vid}{max(jLeftArm)}];

H_jRightKnee8 {vid} = [H_allJoints8{vid}{min(jRightLeg)}];
H_jRightAnkle8 {vid} = [H_allJoints8{vid}{max(jRightLeg)}];

H_jLeftKnee8 {vid} = [H_allJoints8{vid}{min(jLeftLeg)}];
H_jLeftAnkle8 {vid} = [H_allJoints8{vid}{max(jLeftLeg)}];
        
tmpMat{vid} = FFTJD_normHist;     
    
end


%% Reshaping for remaining pipeline

FFTJD_RightElbow8 = {};
FFTJD_RightElbow8  = reshape(cell2mat(H_jRightElbow8), (numBin), (numel(vidRange)))';

FFTJD_RightWrist8 = {};
FFTJD_RightWrist8  = reshape(cell2mat(H_jRightWrist8), (numBin), (numel(vidRange)))';

FFTJD_RightKnee8 = {};
FFTJD_RightKnee8  = reshape(cell2mat(H_jRightKnee8), (numBin), (numel(vidRange)))';

FFTJD_RightAnkle8 = {};
FFTJD_RightAnkle8  = reshape(cell2mat(H_jRightAnkle8), (numBin), (numel(vidRange)))';


FFTJD_LeftElbow8 = {};
FFTJD_LeftElbow8  = reshape(cell2mat(H_jLeftElbow8), (numBin), (numel(vidRange)))';

FFTJD_LeftWrist8 = {};
FFTJD_LeftWrist8  = reshape(cell2mat(H_jLeftWrist8), (numBin), (numel(vidRange)))';

FFTJD_LeftKnee8 = {};
FFTJD_LeftKnee8  = reshape(cell2mat(H_jLeftKnee8), (numBin), (numel(vidRange)))';

FFTJD_LeftAnkle8 = {};
FFTJD_LeftAnkle8  = reshape(cell2mat(H_jLeftAnkle8), (numBin), (numel(vidRange)))';

% FFTJD_FullBody8 = {};
% FFTJD_FullBody8  = reshape(cell2mat(H_jFullBody8), (numBin), (numel(vidRange)))';


%% Export all Histogram data (HOJD2D) for remaining pipeline
save(['25J_RVI_38_FFT_JD_' num2str(numBin) 'bins' '.mat'], 'FFTJD_RightElbow8', 'FFTJD_RightWrist8', 'FFTJD_RightKnee8', 'FFTJD_RightAnkle8', 'FFTJD_LeftElbow8', 'FFTJD_LeftWrist8', 'FFTJD_LeftKnee8', 'FFTJD_LeftAnkle8');