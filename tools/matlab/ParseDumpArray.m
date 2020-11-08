function out = ParseDumpArray(array)
    if (size(array,2) ~= 7) 
        error('Incorrect dump array size when parsing');
    end

    out.time = array(:,1);
    out.throttle = array(:,2);
    out.steering = array(:,3);
    out.out_throttle = array(:,4);
    out.out_steering = array(:,5);
    out.encoder_front = array(:,6);
    out.encoder_back = array(:,7);

end
