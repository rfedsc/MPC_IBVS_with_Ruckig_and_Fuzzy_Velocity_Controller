function [max_acc, max_jerk, fis] = fuzzy_velocity_controller_V3(e, de, a_rated)

    fis = mamfis('Name', 'fuzzy_velocity_controller');

    fis = addInput(fis, [0 1], 'Name', 'e');
    fis = addMF(fis, 'e', 'gbellmf', [0.2 4 0.1], 'Name', 'Tiny');
    fis = addMF(fis, 'e', 'gbellmf', [0.2 4 0.4], 'Name', 'Medium');
    fis = addMF(fis, 'e', 'gbellmf', [0.2 4 0.75], 'Name', 'Large');

    fis = addInput(fis, [-1 1], 'Name', 'de');
    fis = addMF(fis, 'de', 'gaussmf', [0.3 -0.6], 'Name', 'Negative');
    fis = addMF(fis, 'de', 'gaussmf', [0.3 0], 'Name', 'Zero');
    fis = addMF(fis, 'de', 'gaussmf', [0.3 0.6], 'Name', 'Positive');

    fis = addOutput(fis, [5 a_rated], 'Name', 'max_acc');
    fis = addMF(fis, 'max_acc', 'gaussmf', [0.1*a_rated 0.5*a_rated], 'Name', 'Low');
    fis = addMF(fis, 'max_acc', 'gaussmf', [0.1*a_rated 0.7*a_rated], 'Name', 'Medium');
    fis = addMF(fis, 'max_acc', 'gaussmf', [0.1*a_rated 0.9*a_rated], 'Name', 'High');

    fis = addOutput(fis, [3*a_rated 10*a_rated], 'Name', 'max_jerk');
    fis = addMF(fis, 'max_jerk', 'gaussmf', [0.1 * 10*a_rated 0.3 * 10*a_rated], 'Name', 'Smooth');
    fis = addMF(fis, 'max_jerk', 'gaussmf', [0.1 * 10*a_rated 0.6 * 10*a_rated], 'Name', 'Balanced');
    fis = addMF(fis, 'max_jerk', 'gaussmf', [0.1 * 10*a_rated 1.0 * 10*a_rated], 'Name', 'Responsive');

    rulelist = [
        3, 3, 3, 3, 1, 1;
        3, 2, 3, 2, 1, 1;
        3, 1, 2, 2, 1, 1;
        
        2, 3, 2, 3, 1, 1;
        2, 2, 2, 2, 1, 1;
        2, 1, 1, 1, 1, 1;
        
        1, 3, 2, 2, 1, 1;
        1, 2, 1, 1, 1, 1;
        1, 1, 1, 1, 1, 1;
    ];

    fis = addRule(fis, rulelist);

    fis.DefuzzMethod = 'centroid';

    e_n = max(0, min(1, abs(e)));
    de_n = max(-1, min(1, de));

    outputs = evalfis([e_n, de_n], fis);

    max_acc = outputs(1);
    max_jerk = outputs(2);

end