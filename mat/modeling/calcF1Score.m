function f1 = calcF1Score(precision,recall)
if (precision == 0) || (recall == 0)
    f1 = 0;
else
    f1 = 2*(precision*recall)/(precision+recall);
end
end