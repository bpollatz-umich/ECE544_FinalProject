function rrtPath = rrtPlanner(startPos,goalPos,obstacles,wsBounds,displayPath)
arguments
    startPos (1,3) double = [2,2,0] % x, y, theta
    goalPos (1,3) double = startPos % x, y, theta
    obstacles (:,4) double = [1,0,0,0.1] % [ID, Cx, Cy, S]
    wsBounds (2,2) double = [0,50;0,50]  % [xmin, xmax; ymin, ymax]
    displayPath (1,1) logical = false % true to generate figures
end % arg

%% Initialize

% Set parameters and given problem information
goalTol = 2; % Tolerance radius of reaching goal position
angTol = 0.5; % anglular tolerance within goal
stepSize = 0.3; % fixed delta to extend tree by
neighborSearchRadius = 5; % radius of nearby neighbors - must be larger than stepSize
maxNodes = 5000; % Memory limit for node locations
currNode = 2;
gTree = zeros(maxNodes,5,'double'); % x,y,theta of current; parent node idx, cost
gTree(1,1:3) = startPos;
gTree(:,5) = inf; % setting all costs to inf
gTree(1,5) = 0; % Initial cost is 0
looping = true;

%% 1. Random Sampling

while(looping && currNode < maxNodes)
    % To get a random sample point, we get a random (uniform distribution)
    % value from the rand function, and get a point (x,y) within the workspace
    % range. This is set up dynamically, allowing workspace to be adjusted in
    % future.
    newTarget = [(wsBounds(1,2)-wsBounds(1,1))*rand()+wsBounds(1,1), ...
        (wsBounds(2,2)-wsBounds(2,1))*rand()+wsBounds(2,1),...
        2*pi*rand()-pi]; % random x,y,heading, heading ranges +/- pi
    %% 2. Nearest Node

    % The nearest node is calculated by getting the euclidian distance between
    % the subject node and all nodes in existence. The closest node is returned
    % as bestNode, which is bestDist away from the current node.
    bestDist = inf;
    bestNode = 1;
    for idx = 1:currNode-1
        currDist = norm(gTree(idx,1:2)-newTarget(1:2));
        if(currDist < bestDist)
            bestDist = currDist;
            bestNode = idx;
        end
    end
    %% 3. Tree Extension

    % Extend the tree by stepSize distance towards the new target node.
    newTheta = atan2(newTarget(2)-gTree(bestNode,2),newTarget(1)-gTree(bestNode,1));
    newNode = [gTree(bestNode,1:2) + (newTarget(1:2)-gTree(bestNode,1:2))*(stepSize/(eps+bestDist)),newTheta]; % EPS to prevent divide-by-zero
    % when bestDist = 0 (meaning we are repeating a node)
    %% 4. Collision Checking

    % Using the provided function from lecture under HelperFunctions. Steps
    % 5-8 are nested in order to prevent executing further steps if node is
    % rejected.
    collisionFlg = checkCollide(newNode(1:2),gTree(bestNode,1:2),obstacles,wsBounds);
    if(~collisionFlg)
        gTree(currNode,1:3) = newNode;
        %gTree(currNode,4) = bestNode;
        %% 5. Neighbor Search

        % This section finds the # of neighbors within the neighbor radius, and
        % returns a vector of indices within gTree
        neighborRadius = zeros(currNode,5); % gTreeIdx, Neighbor coordinate x,y, neighbor cost, proposed new cost (to be set in step #6)
        currNeighbor = 1;
        for idx = 1:currNode-1
            if(norm(gTree(idx,1:2)-newNode(1:2)) <= neighborSearchRadius)
                neighborRadius(currNeighbor,1) = idx;
                neighborRadius(currNeighbor,2:4) = gTree(idx,[1:2,5]); % sends neighbor and cost only (don't need parent)
                currNeighbor = currNeighbor + 1;
            end % if
        end % for
        neighborRadius(currNeighbor:end,:) = [];
        clear currNeighbor;
        %% 6. Choose Parent

        % Now, we choose a new parent from the list of neighbors nearby, using
        % their applicable cost values.
        for idx = 1:size(neighborRadius,1)
            neighborRadius(idx,5) = neighborRadius(idx,4) + norm(neighborRadius(idx,2:3)-newNode(1:2));
        end % for
        looping2 = true;
        while(looping2)
            [newScore,newParentIdx] = min(neighborRadius(:,5));
            if(newScore == inf) % no parent found
                looping2 = false;
                collisionFlg = true; % reject this
            elseif(~checkCollide(neighborRadius(newParentIdx,2:3),newNode(1:2),obstacles,wsBounds))
                looping2 = false;
            else
                neighborRadius(newParentIdx,5) = inf;
            end % if
        end % while
        if(~collisionFlg)
            gTree(currNode,4) = neighborRadius(newParentIdx,1);
            gTree(currNode,5) = newScore;

            %% 7. Rewiring

            % Now that the new node is optimal, now check the neighbors to see if any
            % can be improved.
            for idx = 1:size(neighborRadius,1)
                propScore = newScore + norm(neighborRadius(idx,2:3)-newNode(1:2));
                if(neighborRadius(idx,4) > propScore && ...
                        ~checkCollide(neighborRadius(idx,2:3),newNode(1:2),obstacles,wsBounds))
                    gTree(neighborRadius(idx,1),4:5) = [currNode,propScore];
                end % if
            end % for
        end % if
        %% 8. Termination Condition
        if(norm(newNode(1:2)-goalPos(1:2)) <= goalTol && ...
                abs(angdiff(newNode(3),goalPos(3))) <= angTol) % Completion condition
            looping = false;
        end % if
        currNode = currNode + 1; % incrementing this now instead of earlier
    end % if (~collisionflg)
end % while
treeSize = currNode-1;

%% Path Smoothing (B-Splines)
% Now, I will take my path, generated as nodeList, and smooth this using
% B-Spline trajectory. To save time, the spline function in Matlab is used.
% % Path Extraction
nodeList = treeSize; % Path indices
curr = treeSize;
while curr > 1
    curr = gTree(curr, 4);
    nodeList = [curr, nodeList];
end

xvar = gTree(nodeList,1); % x coords of path
yvar = gTree(nodeList,2); % y coords of path
resolution = linspace(1,size(nodeList,2),500);
% Per lecture 4, x and y need to be handled separately
xvar = spline(1:size(nodeList,2),xvar,resolution);
yvar = spline(1:size(nodeList,2),yvar,resolution);

% Calculate the heading - the tangent of the path
dx = diff(xvar);
dy = diff(yvar);
tvar = atan2(dy,dx);
tvar = [tvar tvar(end)]; % fixes truncation of last point
rrtPath = [xvar,yvar,tvar];

%% Display Results
if(displayPath)
    figure;
    title("RRT* Path Planning");
    xlabel('X (m)');
    ylabel('Y (m)');
    hold on;
    grid on;
    axis square;

    % Obstacle Plotting:
    % The "rectangle" function is used as previously used in lecture 8 - must translate to lower left corner, then w/h is same:
    for k = 1:size(obstacles,1)
        rectangle('Position', [obstacles(k,2)-(obstacles(k,4)/2),obstacles(k,3)-(obstacles(k,4)/2),obstacles(k,4),obstacles(k,4)],...
            'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'k');
    end % for

    % Target Plotting
    plot(startPos(1),startPos(2),"MarkerSize",10,"Marker",".");
    quiver(startPos(1),startPos(2),cos(startPos(3)),sin(startPos(3)),2,'g','LineWidth',2);
    plot(goalPos(1),goalPos(2),"MarkerSize",10,"Marker",".");
    quiver(goalPos(1),goalPos(2),cos(goalPos(3)),sin(goalPos(3)),2,'r','LineWidth',2);

    % Tree Plotting
    for idx = 2:treeSize
        currParent = gTree(idx,4); % current parent to plot
        if(currParent) > 0
            line([gTree(idx,1),gTree(currParent,1)],...
                [gTree(idx,2),gTree(currParent,2)],...
                'color',[0.7,0.7,0.7],'LineWidth',0.7);
        end % if
    end % for

    % Plot path along tree
    for idx = 1:length(nodeList)-1
        nodeA = nodeList(idx);
        nodeB = nodeList(idx+1);
        line([gTree(nodeA,1), gTree(nodeB,1)], ...
            [gTree(nodeA,2), gTree(nodeB,2)], ...
            'Color', [0.1, 0.1, 0.1], 'LineWidth', 2);
    end % for
    figure;
    title("Smoothing using B-Splines");
    xlabel('X (m)');
    ylabel('Y (m)');
    hold on;
    grid on;
    axis square;

    % Plot original path
    for idx = 1:size(nodeList,2)-1
        nodeA = nodeList(idx);
        nodeB = nodeList(idx+1);
        line([gTree(nodeA,1), gTree(nodeB,1)], ...
            [gTree(nodeA,2), gTree(nodeB,2)], ...
            "Color",'r','LineWidth', 4);
    end % for

    % Plot smoothed path
    plot(xvar,yvar,"k-","LineWidth",1.5);
    legend("RRT* Path","Smoothed B-Spline Trajectory");
end % if
%% Helper Functions

% checkCollide returns a flag indicating if an obstacle or world boundaries
% collide with a proposed node. Node is 1x2, obstalce is rx4, and bounds
% are 2x2

    function hit = checkCollide(p1, p2, obstacles, wsBounds)
        hit = false;
        margin = 0.1; % Small inflation buffer to prevent corner clipping

        % Number of samples based on distance to ensure we don't "jump" over corners
        d = norm(p2-p1);
        numChecks = ceil(d / 0.05); % Check every 0.05 units

        for t = linspace(0, 1, numChecks)
            p = (1-t)*p1 + t*p2;

            % Boundary Check
            if p(1) < wsBounds(1,1)+margin || p(1) > wsBounds(1,2)-margin || ...
                    p(2) < wsBounds(2,1)+margin || p(2) > wsBounds(2,2)-margin
                hit = true; return;
            end

            % Obstacle Check
            for i = 1:size(obstacles,1)
                halfS = (obstacles(i,4)/2) + margin; % Inflated side
                if abs(p(1) - obstacles(i,2)) <= halfS && ...
                        abs(p(2) - obstacles(i,3)) <= halfS
                    hit = true; return;
                end % if
            end % for
        end % for
    end % function
end % main function