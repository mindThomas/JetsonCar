function plot_scan(scan, varargin)
    if (length(varargin))
        tf = varargin{1};
    else
        tf = eye(4);
    end

    heading = tf2heading(tf);

    angle_span = scan.AngleMax - scan.AngleMin;
    num_angles = (angle_span / scan.AngleIncrement + 1);    
    angles = linspace(scan.AngleMin, scan.AngleMax, num_angles)';
    ranges = scan.Ranges;
    if (length(angles) ~= length(ranges))
        error('Mismatch in vector lengths');
    end
    
    x = cos(heading + angles) .* ranges + tf(1,4);
    y = sin(heading + angles) .* ranges + tf(2,4);
        
    plt_idx = find(ranges >= scan.RangeMin & ranges <= scan.RangeMax);
    
    plot(x(plt_idx), y(plt_idx), '*');
end