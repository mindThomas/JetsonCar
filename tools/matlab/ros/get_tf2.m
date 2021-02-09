function tf2 = get_tf2(bag, time, from, to)
    if (~isa(time, 'ros.msg.Time'))
        time = rostime(time);
    end

    tf2 = [];    
    if (canTransform(bag, to, from, time))
        tf2 = getTransform(bag, to, from, time);        
    end        
end