function [out] = printAllBlockNames(signals)
  for i = 1:length(signals)
    blockName = signals(i).blockName;
    if isempty(blockName)
      blockName = '[empty]';
    end
    disp(blockName);
  end
end
