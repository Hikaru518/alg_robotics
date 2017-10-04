    //obstacle configuration space 1
    std::vector<Rectangle> obstacles;
    Rectangle obstacle;
    obstacle.x = -0.7;
    obstacle.y = 0.0;
    obstacle.width = 0.1;
    obstacle.height = 2.0;
    obstacles.push_back(obstacle);

    Rectangle obstacle2;
    obstacle2.x = -0.3;
    obstacle2.y = -2.0;
    obstacle2.width = 0.1;
    obstacle2.height = 2.0;
    obstacles.push_back(obstacle2);

    Rectangle obstacle3;
    obstacle3.x = 0.1;
    obstacle3.y = 0.0;
    obstacle3.width = 0.1;
    obstacle3.height = 2.0;
    obstacles.push_back(obstacle3);

    Rectangle obstacle4;
    obstacle4.x = 0.5;
    obstacle4.y = -2.0;
    obstacle4.width = 0.1;
    obstacle4.height = 2.0;
    obstacles.push_back(obstacle4);

    ax.add_patch(patches.Polygon([(-0.7,2.0),(-0.7,0.0),(-0.6,0.0),(-0.6,2.0)], fill=True, color='0.20'))
    ax.add_patch(patches.Polygon([(-0.3,0.0),(-0.3,-2.0),(-0.2,-2.0),(-0.2,0.0)], fill=True, color='0.20'))
    ax.add_patch(patches.Polygon([(0.1,2.0),(0.1,0.0),(0.2,0.0),(0.2,2.0)], fill=True, color='0.20'))
    ax.add_patch(patches.Polygon([(0.5,0.0),(0.5,-2.0),(0.6,-2.0),(0.6,0.0)], fill=True, color='0.20'))

    ax.add_patch(patches.Polygon([(-2.0,-1.2),(-2.0,-1.3),(0.9,-1.3),(0.9,-1.2)], fill=True, color='0.20'))
    ax.add_patch(patches.Polygon([(0.8,0.3),(0.8,-1.2),(0.9,-1.2),(0.9,0.3)], fill=True, color='0.20'))
    ax.add_patch(patches.Polygon([(0.0,0.4),(0.0,0.3),(0.9,0.3),(0.9,0.4)], fill=True, color='0.20'))
    ax.add_patch(patches.Polygon([(-0.9,1.3),(-0.9,1.2),(2.0,1.2),(2.0,1.3)], fill=True, color='0.20'))
    ax.add_patch(patches.Polygon([(-0.9,1.2),(-0.9,-0.3),(-0.8,-0.3),(-0.8,1.2)], fill=True, color='0.20'))
    ax.add_patch(patches.Polygon([(-0.9,-0.3),(-0.9,-0.4),(-0.0,-0.4),(-0.0,-0.3)], fill=True, color='0.20'))


    //obstacle configuration space 2
    std::vector<Rectangle> obstacles;
    Rectangle obstacle;
    obstacle.x = -2.0;
    obstacle.y = -1.3;
    obstacle.width = 2.9;
    obstacle.height = 0.1;
    obstacles.push_back(obstacle);

    Rectangle obstacle2;
    obstacle2.x = 0.8;
    obstacle2.y = -1.2;
    obstacle2.width = 0.1;
    obstacle2.height = 1.5;
    obstacles.push_back(obstacle2);

    Rectangle obstacle3;
    obstacle3.x = 0.0;
    obstacle3.y = 0.3;
    obstacle3.width = 0.9;
    obstacle3.height = 0.1;
    obstacles.push_back(obstacle3);

    Rectangle obstacle4;
    obstacle4.x = -0.9;
    obstacle4.y = 1.2;
    obstacle4.width = 2.9;
    obstacle4.height = 0.1;
    obstacles.push_back(obstacle4);

    Rectangle obstacle5;
    obstacle5.x = -0.9;
    obstacle5.y = -0.3;
    obstacle5.width = 0.1;
    obstacle5.height = 1.5;
    obstacles.push_back(obstacle5);

    Rectangle obstacle6;
    obstacle6.x = -0.9;
    obstacle6.y = -0.4;
    obstacle6.width = 0.9;
    obstacle6.height = 0.1;
    obstacles.push_back(obstacle6);







