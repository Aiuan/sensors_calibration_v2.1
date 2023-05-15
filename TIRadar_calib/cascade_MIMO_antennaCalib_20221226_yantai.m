%  Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
%
%
%   Redistribution and use in source and binary forms, with or without
%   modification, are permitted provided that the following conditions
%   are met:
%
%     Redistributions of source code must retain the above copyright
%     notice, this list of conditions and the following disclaimer.
%
%     Redistributions in binary form must reproduce the above copyright
%     notice, this list of conditions and the following disclaimer in the
%     documentation and/or other materials provided with the
%     distribution.
%
%     Neither the name of Texas Instruments Incorporated nor the names of
%     its contributors may be used to endorse or promote products derived
%     from this software without specific prior written permission.
%
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
%   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
%   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
%   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
%   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
%   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
%   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
%   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
%   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
%

% cascade_MIMO_antennaCalib.m
%  
% Top level main test chain to perform antenna calibration. 
% genCalibrationMatrixCascade module is first initialized before
% actually used in the chain. The output is saved to a .mat file to be used
% later in data processing

clearvars;
close all;
clc;

% add path
addpath('./@Module/');
addpath('./@genCalibrationMatrixCascade/');
addpath('./cascade_json_parser/');
addpath('./dataParse/');
addpath('./math/');

% output_folder
output_folder = './20221226_yantai';
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
    fprintf('create %s\n', output_folder);
end
% input & output
output_path = fullfile(output_folder, 'mode1_256x128.mat');
dataFolder_calib_data = 'F:\sensors_calibration_v2\rawdata\20221217_calibration\TIRadar_calibration\TIRadar\20221217_160058_mode1_calib\20221217_160043_mode1_10';

% output_path = fullfile(output_folder, 'mode2_256x128.mat');
% dataFolder_calib_data = 'F:\sensors_calibration_v2\rawdata\20221217_calibration\TIRadar_calibration\TIRadar\20221217_160154_mode2_calib\20221217_160136_mode2_10';

% output_path = fullfile(output_folder, 'mode3_128x255.mat');
% dataFolder_calib_data = 'F:\sensors_calibration_v2\rawdata\20221217_calibration\TIRadar_calibration\TIRadar\20221217_160242_mode3_calib\20221217_160227_mode3_10';

% output_path = fullfile(output_folder, 'mode4_512x64.mat');
% dataFolder_calib_data = 'F:\sensors_calibration_v2\rawdata\20221217_calibration\TIRadar_calibration\TIRadar\20221217_160332_mode4_calib\20221217_160319_mode4_10';


targetRange = 5; %target aproximate distance for local maximum search
dataPlatform = 'TDA2';

%parameter file name for the test
pathGenParaFile = './generateClibrationMatrix_param.m';

%important to clear the same.m file, since Matlab does not clear cache
%automatically
clear(pathGenParaFile);
%generate parameter file for the test to run
parameter_file_gen_antennaCalib_json(dataFolder_calib_data, pathGenParaFile, dataPlatform);


genCalibrationMatrixObj      = genCalibrationMatrixCascade('pfile', pathGenParaFile,...
    'calibrateFileName',dataFolder_calib_data, 'targetRange', targetRange);

[fileIdx_unique] = getUniqueFileIdx(dataFolder_calib_data);
[fileNameStruct] = getBinFileNames_withIdx(dataFolder_calib_data, fileIdx_unique{1})        
genCalibrationMatrixObj.binDataFile = fileNameStruct;% dataFolder_calib_data;%[dataFolder_calib_data listing.name];

if length(genCalibrationMatrixObj.TxToEnable)< 12
    %it is important to know that all 12 TX needs to be turned on in the MIMO mode to generate a correct calibration matrix. 
    error('This data set cannot be used for calibration, all 12 channels should be enabled');
end

%calibrateValFileNameSave: file name to save calibration results. This file
%will be saved in "dataFolder_calib" after running calibration
calibrateValFileNameSave = output_path;

%use second frame for calibration 
genCalibrationMatrixObj.frameIdx = 2;

calibResult = dataPath(genCalibrationMatrixObj);
RangeMat = calibResult.RangeMat;
targetRange_est = (floor(mean(RangeMat(:))/genCalibrationMatrixObj.calibrationInterp))...
    *genCalibrationMatrixObj.rangeResolution;
disp(['Target is estimated at range ' num2str(targetRange_est)]);

figure(1);
plot(RangeMat(:));grid on;
title('peak index across all channels')

figure(2);
plot(squeeze(abs(calibResult.Rx_fft(:,1,:))))
%just to make it compatible with old data
params.Slope_MHzperus = genCalibrationMatrixObj.Slope_calib/1e12;
params.Sampling_Rate_sps = genCalibrationMatrixObj.Sampling_Rate_sps;
%save the calibration data
save(calibrateValFileNameSave, 'calibResult', 'params');
