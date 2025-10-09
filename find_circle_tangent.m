function [tangent_point, distance] = find_circle_tangent(A, C, r, selector)
% FIND_CIRCLE_TANGENT Calculates a tangent point on a circle from an external point.
%
% SYNTAX:
%   [tangent_point, distance] = find_circle_tangent(A, C, r)
%   [tangent_point, distance] = find_circle_tangent(A, C, r, selector)
%
% DESCRIPTION:
%   This function determines the coordinates of a tangent point on a circle
%   from a given external point A. It also returns the distance from A
%   to that tangent point. A circle has two possible tangent lines from any
%   external point. The 'selector' input is used to choose between them.
%
% INPUTS:
%   A             - A 1x2 or 2x1 vector for the external point coords [Ax, Ay].
%   C             - A 1x2 or 2x1 vector for the circle's center [Cx, Cy].
%   r             - A positive scalar for the circle's radius.
%   selector      - (Optional) A scalar to select which tangent to return.
%                   Use 1 (default) or -1.
%
% OUTPUTS:
%   tangent_point - A 1x2 vector [Tx, Ty] for the tangent point on the circle.
%                   Returns [NaN, NaN] if point A is inside the circle.
%   distance      - The scalar distance from point A to the tangent point.
%                   Returns NaN if point A is inside the circle.
%
% EXAMPLE:
%   A = [10, 3];
%   C = [2, 1];
%   r = 4;
%   [T1, dist1] = find_circle_tangent(A, C, r, 1);
%   [T2, dist2] = find_circle_tangent(A, C, r, -1);
%
%   figure;
%   hold on; axis equal; grid on;
%   viscircles(C, r, 'Color', 'b'); % The circle
%   plot(A(1), A(2), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Point A');
%   plot(C(1), C(2), 'bs', 'MarkerFaceColor', 'b', 'DisplayName', 'Center C');
%   plot(T1(1), T1(2), 'gx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Tangent 1');
%   plot(T2(1), T2(2), 'mx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Tangent 2');
%   plot([A(1) T1(1)], [A(2) T1(2)], 'g--');
%   plot([A(1) T2(1)], [A(2) T2(2)], 'm--');
%   legend('show', 'Location', 'northwest');
%   title('Tangents from a Point to a Circle');

% --- Input Validation & Defaults ---
if nargin < 4
    selector = 1; % Default to one of the tangents
end

% Ensure inputs are 1x2 row vectors for consistent calculations
A = A(:)';
C = C(:)';

if numel(A) ~= 2 || numel(C) ~= 2
    error('Points A and C must be 2-element vectors.');
end
if ~isscalar(r) || r <= 0
    error('Radius r must be a positive scalar.');
end
if ~ismember(selector, [1, -1])
    error('Selector must be either 1 or -1.');
end

% --- Calculations ---

% Vector from the circle's center C to the external point A
vec_CA = A - C;

% Squared distance from C to A
d_sq = dot(vec_CA, vec_CA);

% Define a small tolerance for floating-point comparisons
tolerance = 1e-9;

% Check if the point A is inside or on the circle
if d_sq < r^2 - tolerance
    warning('Point is inside the circle; no real tangent exists.');
    tangent_point = [NaN, NaN];
    distance = NaN;
    return;
elseif abs(d_sq - r^2) <= tolerance
    % Point is on the circle, the tangent point is A itself
    tangent_point = A;
    distance = 0;
    return;
end

% The distance from A to the tangent point T is L, where L^2 = d^2 - r^2
distance = sqrt(d_sq - r^2);

% We find the tangent point T using a geometric construction that avoids
% trigonometric functions, which is more robust. The formula is:
% T = C + (r^2/d^2)*(A-C)  +/-  (r*L/d^2)*Rot_90(A-C)

% First component of the vector CT (along the line CA)
term1 = (r^2 / d_sq) * vec_CA;

% Second component of the vector CT (perpendicular to the line CA)
% A 2D vector (x,y) rotated by +90 degrees is (-y,x)
rotated_vec = [-vec_CA(2), vec_CA(1)];
term2 = (r * distance / d_sq) * rotated_vec;

% The vector from the center C to the tangent point T is the sum of the two components.
% The 'selector' determines which perpendicular direction to use (+/-).
vec_CT = term1 + selector * term2;

% The final tangent point is the center C plus the vector CT.
tangent_point = C + vec_CT;

end
