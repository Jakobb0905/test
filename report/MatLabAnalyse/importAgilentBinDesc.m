function [waveformDescription] = importAgilentBinDescr(inputFilename)
% ImportAgilentBinDescr reads the Agilent Binary Waveform file and extracts
% the description of the contents.
%
% [description] = importAgilentBinDescr(inputFilename)
% Description is a structure (could be array) containing information about
% the waveforms stored in inputFilename.

if (~exist(inputFilename))
    error('inputFilename missing.');
end

fileId = fopen(inputFilename, 'r');

% read file header
fileCookie = fread(fileId, 2, 'char');
fileVersion = fread(fileId, 2, 'char');
fileSize = fread(fileId, 1, 'int32');
nWaveforms = fread(fileId, 1, 'int32');

% verify cookie
fileCookie = char(fileCookie');
if (~strcmp(fileCookie, 'AG'))
    fclose(fileId);
    error('Unrecognized file format.');
end

for waveformIndex = 1:nWaveforms
    % read waveform header
    headerSize = fread(fileId, 1, 'int32'); bytesLeft = headerSize - 4;
    waveformDescription(waveformIndex).waveformType = fread(fileId, 1, 'int32'); bytesLeft = bytesLeft - 4;
    waveformDescription(waveformIndex).nWaveformBuffers = fread(fileId, 1, 'int32'); bytesLeft = bytesLeft - 4;
    waveformDescription(waveformIndex).nPoints = fread(fileId, 1, 'int32'); bytesLeft = bytesLeft - 4;
    waveformDescription(waveformIndex).count = fread(fileId, 1, 'int32');  bytesLeft = bytesLeft - 4;
    waveformDescription(waveformIndex).xDisplayRange = fread(fileId, 1, 'float32');  bytesLeft = bytesLeft - 4;
    waveformDescription(waveformIndex).xDisplayOrigin = fread(fileId, 1, 'double');  bytesLeft = bytesLeft - 8;
    waveformDescription(waveformIndex).xIncrement = fread(fileId, 1, 'double');  bytesLeft = bytesLeft - 8;
    waveformDescription(waveformIndex).xOrigin = fread(fileId, 1, 'double');  bytesLeft = bytesLeft - 8;
    waveformDescription(waveformIndex).xUnits = fread(fileId, 1, 'int32');  bytesLeft = bytesLeft - 4;
    waveformDescription(waveformIndex).yUnits = fread(fileId, 1, 'int32');  bytesLeft = bytesLeft - 4;
    waveformDescription(waveformIndex).dateString = char(fread(fileId, 16, 'char')'); bytesLeft = bytesLeft - 16;
    waveformDescription(waveformIndex).timeString = char(fread(fileId, 16, 'char')'); bytesLeft = bytesLeft - 16;
    waveformDescription(waveformIndex).frameString = char(fread(fileId, 24, 'char')'); bytesLeft = bytesLeft - 24;
    waveformDescription(waveformIndex).waveformString = char(fread(fileId, 16, 'char')'); bytesLeft = bytesLeft - 16;
    waveformDescription(waveformIndex).timeTag = fread(fileId, 1, 'double'); bytesLeft = bytesLeft - 8;
    waveformDescription(waveformIndex).segmentIndex = fread(fileId, 1, 'uint32'); bytesLeft = bytesLeft - 4;

    % skip over any remaining data in the header
    fseek(fileId, bytesLeft, 'cof');

    for bufferIndex = 1:waveformDescription(waveformIndex).nWaveformBuffers
        % read waveform buffer header
        headerSize = fread(fileId, 1, 'int32'); bytesLeft = headerSize - 4;
        bufferType = fread(fileId, 1, 'int16'); bytesLeft = bytesLeft - 2;
        bytesPerPoint = fread(fileId, 1, 'int16'); bytesLeft = bytesLeft - 2;
        bufferSize = fread(fileId, 1, 'int32'); bytesLeft = bytesLeft - 4;

        % skip over any remaining data in the header
        fseek(fileId, bytesLeft, 'cof');

        % skip over waveform data
        fseek(fileId, bufferSize, 'cof');
    end
end
fclose(fileId);
