%% Initialization

clear;
close all;

load('C_25J_RVI_38_ProcessedData.mat');
vidRange = 1:38;
numBin = 16;
pwr = 1.5;

jRightArm = 4:5;
jLeftArm = 7:8;
jRightLeg = 11:12;
jLeftLeg = 14:15;
jFullBody = 1:21;

H_allJoints16 = {};
H_jRightElbow16 = {};
H_jRightWrist16 = {};
H_jLeftElbow16 = {};
H_jLeftWrist16 = {};
H_jRightKnee16 = {};
H_jRightAnkle16 = {};
H_jLeftKnee16 = {};
H_jLeftAnkle16 = {};


%% FFTJD

for vid = vidRange
    
    for allJoints = jFullBody
        
        frmLength = length(allFinalPose{vid}(1,:))-1;
        
        FFTJD_curHist = zeros(1,numBin);

        for frm = 2:10:frmLength
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
            
            accBins = 0;
            if f(bb) < maxF*(1^pwr/numBin^pwr)
                bestBin = 1;
                accBins = accBins + (1^pwr/numBin^pwr);
            elseif f(bb) < maxF*(accBins+(2^pwr/numBin^pwr))
                bestBin = 2;
                accBins = accBins + (2^pwr/numBin^pwr);
            elseif f(bb) < maxF*(accBins+(3^pwr/numBin^pwr))
                bestBin = 3;
                accBins = accBins + (3^pwr/numBin^pwr);
            elseif f(bb) < maxF*(accBins+(4^pwr/numBin^pwr))
                bestBin = 4;
                accBins = accBins + (4^pwr/numBin^pwr);
            elseif f(bb) < maxF*(accBins+(5^pwr/numBin^pwr))
                bestBin = 5;
                accBins = accBins + (5^pwr/numBin^pwr);
            elseif f(bb) < maxF*(accBins+(6^pwr/numBin^pwr))
                bestBin = 6;
                accBins = accBins + (6^pwr/numBin^pwr);
            elseif f(bb) < maxF*(accBins+(7^pwr/numBin^pwr))
                bestBin = 7;
                accBins = accBins + (7^pwr/numBin^pwr);
            elseif f(bb) < maxF*(8^pwr/numBin^pwr)
                bestBin = 8;
                accBins = accBins + (8^pwr/numBin^pwr);
            elseif f(bb) < maxF*(accBins+(9^pwr/numBin^pwr))
                bestBin = 9;
                accBins = accBins + (9^pwr/numBin^pwr);
            elseif f(bb) < maxF*(accBins+(10^pwr/numBin^pwr))
                bestBin = 10;
                accBins = accBins + (10^pwr/numBin^pwr);
            elseif f(bb) < maxF*(accBins+(11^pwr/numBin^pwr))
                bestBin = 11;
                accBins = accBins + (11^pwr/numBin^pwr);
            elseif f(bb) < maxF*(accBins+(12^pwr/numBin^pwr))
                bestBin = 12;
                accBins = accBins + (12^pwr/numBin^pwr);
            elseif f(bb) < maxF*(accBins+(13^pwr/numBin^pwr))
                bestBin = 13;
                accBins = accBins + (13^pwr/numBin^pwr);
            elseif f(bb) < maxF*(accBins+(14^pwr/numBin^pwr))
                bestBin = 14;
                accBins = accBins + (14^pwr/numBin^pwr);
            elseif f(bb) < maxF*(accBins+(15^pwr/numBin^pwr))
                bestBin = 15;
                accBins = accBins + (15^pwr/numBin^pwr);
            elseif f(bb) <= maxF
                bestBin = 16;
            end
                        
            
            FFTJD_curHist(bestBin) = FFTJD_curHist(bestBin) + P1(bb);
        end
        
        FFTJD_normHist = FFTJD_curHist/sum(FFTJD_curHist);
        
        H_allJoints16{vid}{allJoints} = FFTJD_normHist;
        
        
    end
    

H_jRightElbow16 {vid} = [H_allJoints16{vid}{min(jRightArm)}];
H_jRightWrist16 {vid} = [H_allJoints16{vid}{max(jRightArm)}];

H_jLeftElbow16 {vid} = [H_allJoints16{vid}{min(jLeftArm)}];
H_jLeftWrist16 {vid} = [H_allJoints16{vid}{max(jLeftArm)}];

H_jRightKnee16 {vid} = [H_allJoints16{vid}{min(jRightLeg)}];
H_jRightAnkle16 {vid} = [H_allJoints16{vid}{max(jRightLeg)}];

H_jLeftKnee16 {vid} = [H_allJoints16{vid}{min(jLeftLeg)}];
H_jLeftAnkle16 {vid} = [H_allJoints16{vid}{max(jLeftLeg)}];
        
    
end


%% Reshaping for remaining pipeline

FFTJD_RightElbow16 = {};
FFTJD_RightElbow16  = reshape(cell2mat(H_jRightElbow16), (numBin), (numel(vidRange)))';

FFTJD_RightWrist16 = {};
FFTJD_RightWrist16  = reshape(cell2mat(H_jRightWrist16), (numBin), (numel(vidRange)))';

FFTJD_RightKnee16 = {};
FFTJD_RightKnee16  = reshape(cell2mat(H_jRightKnee16), (numBin), (numel(vidRange)))';

FFTJD_RightAnkle16 = {};
FFTJD_RightAnkle16  = reshape(cell2mat(H_jRightAnkle16), (numBin), (numel(vidRange)))';


FFTJD_LeftElbow16 = {};
FFTJD_LeftElbow16  = reshape(cell2mat(H_jLeftElbow16), (numBin), (numel(vidRange)))';

FFTJD_LeftWrist16 = {};
FFTJD_LeftWrist16  = reshape(cell2mat(H_jLeftWrist16), (numBin), (numel(vidRange)))';

FFTJD_LeftKnee16 = {};
FFTJD_LeftKnee16  = reshape(cell2mat(H_jLeftKnee16), (numBin), (numel(vidRange)))';

FFTJD_LeftAnkle16 = {};
FFTJD_LeftAnkle16  = reshape(cell2mat(H_jLeftAnkle16), (numBin), (numel(vidRange)))';

% FFTJD_FullBody16 = {};
% FFTJD_FullBody16  = reshape(cell2mat(H_jFullBody16), (numBin), (numel(vidRange)))';


%% Export all Histogram data (HOJD2D) for remaining pipeline
save(['25J_RVI_38_FFT_JD_' num2str(numBin) 'bins' '.mat'], 'FFTJD_RightElbow16', 'FFTJD_RightWrist16', 'FFTJD_RightKnee16', 'FFTJD_RightAnkle16', 'FFTJD_LeftElbow16', 'FFTJD_LeftWrist16', 'FFTJD_LeftKnee16', 'FFTJD_LeftAnkle16');