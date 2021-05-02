function res=getcharfield(payload,field)
  res=cell(size(payload));
  field=strsplit(field,'.');
  for i=1:length(payload)
    v=payload(i);
    for j=1:length(field)
      if isfield(v,field{j})
        v=v.(field{j});
      else
        keyboard;
        v='';
        break
      end
    end
    if ~isempty(v)
      res{i}=v;
    end
  end
end
