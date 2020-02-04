function Output = MisRegVis(Master,Slave,BlockSize)
%  This function takes in master and slave images and return a block
%  blended image for misregisteration analysis
%
% INPUTS:
% Master: master image
% Slave: slave image
% BlockSize: Size of blend block 
%
% OUTPUTS:
% Output: blended image
% 
% 
%
%%%%%%%%%%%%%%%% Author: Muzammil Bashir %%%%%%%%%%%%%%%%
%                Date: 29 July, 2017                    %
%                Organization: Burqstream Technologies  %
%                
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


ImgSize = size(Master);


nBlocks = ceil(ImgSize/BlockSize);

% Computer n blocks need to be further divided. See checkboard
% documentation
nBlocks_2 = ceil(nBlocks/2);          

% Get Checker board pattern
mask = checkerboard(BlockSize,nBlocks_2(1),nBlocks_2(2));

% Remove extra rows and column to match image size
mask= mask(1:ImgSize(1),1:ImgSize(2));

% Convert to discrete logicals
mask = mask>0.5;

% Extract corresponding blocks from master and slave frames
Master2 = Master.*double(mask);
Slave2 = Slave.*double(~mask);

% Merge the blocks
Output = Master2+Slave2;

end