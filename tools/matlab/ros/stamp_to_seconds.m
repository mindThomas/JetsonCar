function time = stamp_to_seconds(Stamp)
    time = double(Stamp.Sec) + 1e-9*double(Stamp.Nsec);
end  