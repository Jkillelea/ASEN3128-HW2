function [data] = getDataByBlockName(signals, blockName)
  for i = 1:length(signals)
    if strcmp(signals(i).blockName, blockName)
      data = signals(i).values;
      return;
    end
  end
  data = NaN;
end
