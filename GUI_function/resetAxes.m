function resetAxes(axes)
    cla(axes,'reset');
    grid(axes,'on');
    hold(axes,'on');
    axis(axes, [0 inf -inf inf]);
end

