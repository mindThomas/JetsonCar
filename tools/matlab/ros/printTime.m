function printTime(time)
    t = datetime(time, 'ConvertFrom', 'epochtime', 'TimeZone', 'UTC');
    t.TimeZone = '+8';

    disp(t);
end