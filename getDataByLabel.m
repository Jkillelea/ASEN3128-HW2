function [data] = getDataByLabel(signals, label)
  for i = 1:length(signals)
    if strcmp(signals(i).label, label)
      data = signals(i).values;
      return;
    end
  end
  data = NaN;
end
