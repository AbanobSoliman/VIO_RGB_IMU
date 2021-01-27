function error_ellipse(pqpos)

    % Inject the gaussian estimated data
    y1 = pqpos(:,1); % East (x)
    y2 = pqpos(:,2); % North (y)
    data = [y1 y2];

    % Calculate the eigenvectors and eigenvalues
    covariance = cov(data);
    [eigenvec, eigenval ] = eig(covariance);

    % Get the index of the largest eigenvector
    [largest_eigenvec_ind_c, ~] = find(eigenval == max(max(eigenval)));
    largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);

    % Get the largest eigenvalue
    largest_eigenval = max(max(eigenval));

    % Get the smallest eigenvector and eigenvalue
    if(largest_eigenvec_ind_c == 1)
        smallest_eigenval = max(eigenval(:,2));
        smallest_eigenvec = eigenvec(:,2);
    else
        smallest_eigenval = max(eigenval(:,1));
        smallest_eigenvec = eigenvec(1,:);
    end

    % Calculate the angle between the x-axis and the largest eigenvector
    angle = atan2(largest_eigenvec(2), largest_eigenvec(1));

    % This angle is between -pi and pi.
    % Let's shift it such that the angle is between 0 and 2pi
    if(angle < 0)
        angle = angle + 2*pi;
    end

    % Get the coordinates of the data mean
    avg = mean(data);

    figure
    % Plot the original data
    plot(data(:,1), data(:,2), '.');
    hold on;
    
    % Get the 90% confidence interval error ellipse
    chisquare_val = 2.145926373;
    theta_grid = linspace(0,2*pi);
    phi = angle;
    X0=avg(1);
    Y0=avg(2);
    a=chisquare_val*sqrt(largest_eigenval);
    b=chisquare_val*sqrt(smallest_eigenval);

    % the ellipse in x and y coordinates 
    ellipse_x_r  = a*cos( theta_grid );
    ellipse_y_r  = b*sin( theta_grid );

    %Define a rotation matrix
    R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];

    %let's rotate the ellipse to some angle phi
    r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;

    % Draw the error ellipse
    plot(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0,'-','Color','r')
    text(r_ellipse(1,1) + X0,r_ellipse(1,2) + Y0,'90%','Color','red')
    hold on;

    % Plot the eigenvectors
    quiver(X0, Y0, largest_eigenvec(1)*sqrt(largest_eigenval), largest_eigenvec(2)*sqrt(largest_eigenval), '-r', 'LineWidth',2);
    quiver(X0, Y0, smallest_eigenvec(1)*sqrt(smallest_eigenval), smallest_eigenvec(2)*sqrt(smallest_eigenval), '-g', 'LineWidth',2);
    hold on;
    
    % Get the 95% confidence interval error ellipse
    chisquare_val = 2.447651936;
    theta_grid = linspace(0,2*pi);
    phi = angle;
    X0=avg(1);
    Y0=avg(2);
    a=chisquare_val*sqrt(largest_eigenval);
    b=chisquare_val*sqrt(smallest_eigenval);

    % the ellipse in x and y coordinates 
    ellipse_x_r  = a*cos( theta_grid );
    ellipse_y_r  = b*sin( theta_grid );

    %Define a rotation matrix
    R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];

    %let's rotate the ellipse to some angle phi
    r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;

    % Draw the error ellipse
    plot(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0,'-','Color','b')
    text(r_ellipse(1,1) + X0,r_ellipse(1,2) + Y0,'95%','Color','blue')
    hold on;

    % Plot the eigenvectors
    quiver(X0, Y0, largest_eigenvec(1)*sqrt(largest_eigenval), largest_eigenvec(2)*sqrt(largest_eigenval), '-p', 'LineWidth',2);
    quiver(X0, Y0, smallest_eigenvec(1)*sqrt(smallest_eigenval), smallest_eigenvec(2)*sqrt(smallest_eigenval), '-m', 'LineWidth',2);
    hold on;
    
    % Get the 99% confidence interval error ellipse
    chisquare_val = 3.034798181;
    theta_grid = linspace(0,2*pi);
    phi = angle;
    X0=avg(1);
    Y0=avg(2);
    a=chisquare_val*sqrt(largest_eigenval);
    b=chisquare_val*sqrt(smallest_eigenval);

    % the ellipse in x and y coordinates 
    ellipse_x_r  = a*cos( theta_grid );
    ellipse_y_r  = b*sin( theta_grid );

    %Define a rotation matrix
    R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];

    %let's rotate the ellipse to some angle phi
    r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;

    % Draw the error ellipse
    plot(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0,'-','Color','g')
    text(r_ellipse(1,1) + X0,r_ellipse(1,2) + Y0,'99%','Color','green')
    hold on;

    % Plot the eigenvectors
    quiver(X0, Y0, largest_eigenvec(1)*sqrt(largest_eigenval), largest_eigenvec(2)*sqrt(largest_eigenval), '-m', 'LineWidth',2);
    quiver(X0, Y0, smallest_eigenvec(1)*sqrt(smallest_eigenval), smallest_eigenvec(2)*sqrt(smallest_eigenval), '-p', 'LineWidth',2);
    hold on;

    % Set the axis labels
    xlabel('X (m)')
    ylabel('Y (m)')
    title('Covariance Error Ellipses on Fused X-Y 2D - Path with eigen-vectors')
    grid on
end
