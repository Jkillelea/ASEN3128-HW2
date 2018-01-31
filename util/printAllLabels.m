function [out] = printAllLabels(signals)
  for i = 1:length(signals)
    label = signals(i).label;
    if isempty(label)
      label = '[empty]';
    end
    disp(label);
  end
end
