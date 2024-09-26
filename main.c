#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include <stdbool.h>
#define N 5
#define P 0.2
#define UCS 0
#define A_STAR 1


/* Maze Grid */
struct State ** gp_grid = NULL;


/* Search Frontier */
struct Node * gp_search_frontier_head = NULL;
int g_search_frontier_nodes_count = 0;
int g_number_of_paths_to_nodes_created = 0;


/* Closed Set */
struct Node * gp_closed_set_head = NULL;
int g_closed_set_nodes_count = 0;


/* struct State
 *
 * value = State value created random when we create the grid.
 * is_free = Shows if this p_state is free to access or is blocked.
 * distance_from_initial_state = distance/cost p_state has in relation to p_state which search begins.
 *
 */
struct State{
    int value;
    int is_free;
    double distance_from_initial_state;
};


/* struct Node
 *
 * i,j  = grid coordinates where the p_state that node keeps is positioned.
 * p_next = p_next node when we add this node in a linked list.
 * p_predecessor = p_state parent in order to print the solution when its found.
 * 
 */
struct Node{
    int i, j ;
    struct State * p_state;
    struct Node * p_next;
    struct Node * p_predecessor;
};





/* Methods Declarations */

struct Node * get_node_with_minimum_distance_state_in_search_frontier(void);
int is_node_with_goal_state(struct Node*,const int*,const int*,int*);
int initial_state_is_goal_state(const int*, const int*, const int*);
struct Node * Uniform_Cost_Search_Algorithm(int*,int*,int*,int*);
struct Node * A_STAR_algorithm(int*,int*,int*,int*);
int state_is_in_search_frontier(struct State *p_state_to_check);
int state_is_in_closed_set(struct State *p_state_to_check);
int read_user_input(int*,int*,int*);
double heuristic_function(int,int,int*,int*);
double min(double,double);
void create_and_add_node_to_a_set(struct Node **set_head, struct State *p_state, int *nodes_count, int i, int j
        , struct Node*);
void add_children_to_search_frontier(int,int,struct Node*,int*, int*,int);
void UCS_OR_A_STAR_solution_handler(struct Node*,int,double );
void add_Node_in_closed_set(struct Node *p_node);
void algorithm_selection(int, int*, int*, int*);
void free_linked_list(struct Node**, int*);
void switch_case_A_STAR(int*,int*,int*);
void switch_case_UCS(int*, int*, int*);
void switch_case_BOTH(int*,int*,int*);
void free_states(struct State** grid);
void print_nodes_in_search_frontier(void);
void print_nodes_in_closed_set(void);
void reset_distances_grid(void);
void program_start_prints(void);
void create_grid(void);
void print_grid(void);




int main(void)
{

    int s[2] , g_1[2] , g_2[2], algorithm_ID;
    program_start_prints();
    algorithm_ID = read_user_input(s, g_1, g_2);
    algorithm_selection(algorithm_ID, s, g_1, g_2);
    printf("||  Program Finished  ||\n");

    free_linked_list(&gp_search_frontier_head, &g_search_frontier_nodes_count);
    free_linked_list(&gp_closed_set_head, &g_closed_set_nodes_count);
    free_states(gp_grid);
    
    return 0;
}





///////////////////////////////////////////////////////////////////////////////////////

void reset_distances_grid(void)
{
    for(int i=0; i<N; i++)
        for(int j=0; j<N; j++)
            gp_grid[i][j].distance_from_initial_state = 0;
}

void program_start_prints(void)
{
    printf("||    Maze Searching Program Started   ||\n\n");
    printf("Preset Settings :\nMatrix Size N = %d\nProbability for blocked state in grid P = %lf\n", N, P);
    printf("\nGenerating Grid....\n\n");
    create_grid();
    print_grid();
}

void create_grid(void)
{
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    srand(current_time.tv_usec);

    gp_grid = (struct State **) malloc(N * sizeof (struct State*));
    for(int i=0; i<N; i++)
    {
        gp_grid[i] = (struct State *) malloc(N * sizeof (struct State));
        for(int j=0; j<N; j++)
        {
            gp_grid[i][j].value = rand() % 4 + 1;
            gp_grid[i][j].distance_from_initial_state = 0;
            int is_blocked_state_probability = rand() % 100 ;
            gp_grid[i][j].is_free = (is_blocked_state_probability < P * 100) ? false : true ;
        }
    }
}

void print_grid(void)
{
    for(int i=0; i<N; i++)
    {
        for(int j=0; j<N; j++)
        {
            char* access = gp_grid[i][j].is_free ? "FREE   " : "BLOCKED" ;
            printf("||(%d,%d) Value:%d   %s  ||     ", i, j, gp_grid[i][j].value, access);
        }
        printf("\n\n");
    }
}

int read_user_input(int* initial_state_S, int* final_state_G1, int* final_state_G2) {
    printf("Input the coordinates of initial_state , final_state_g1 , final_state_g2\n");
    printf("---------------------------\nNote that coordinates that lead to a blocked p_state "
           "(according to the grid created above) cannot "
           "be given as input!\nThat will lead to program failure\n---------------------------");
    printf("\nInput the coordinates of the initial state S:\n");
    printf("[Format -> 'x,y']\n");
    printf("Answer:");
    scanf("%d,%d", &initial_state_S[0], &initial_state_S[1]);
    printf("\nStarting Position S selected as the (%d,%d)\n-----\n", initial_state_S[0], initial_state_S[1]);
    printf("Input G1 final state coordinates:\n");
    printf("[Format -> 'x,y']\n");
    printf("Answer:");
    scanf("%d,%d", &final_state_G1[0], &final_state_G1[1]);
    printf("\nFinal State G1 selected as the (%d,%d)\n-----\n", final_state_G1[0], final_state_G1[1]);
    printf("Input the G2 Final State coordinates:\n");
    printf("[Format -> 'x,y']\n");
    printf("Answer:");
    scanf("%d,%d", &final_state_G2[0], &final_state_G2[1]);
    printf("\nFinal State G1 selected as the (%d,%d)\n-----\n", final_state_G2[0], final_state_G2[1]);
    printf("\nSelect what method is to be used:           [Input corresponding number]\n");
    printf("1) Uniform Cost Search [UCS]\n");
    printf("2) A*:\n");
    printf("3) Both:\n");
    printf("Answer:");
    int method_ID;
    scanf("%d",&method_ID);
    return method_ID;
}

void algorithm_selection(int method_ID, int* initial_state_S, int* final_state_G1, int* final_state_G2)
{
    switch (method_ID)
    {
        case 1:
            switch_case_UCS(initial_state_S,final_state_G1,final_state_G2);
            break;
        case 2:
            switch_case_A_STAR(initial_state_S,final_state_G1,final_state_G2);
            break;
        case 3:
            switch_case_BOTH(initial_state_S,final_state_G1,final_state_G2);
            break;
        default:
            printf("Wrong method ID number inputted!\n");
            printf("Error!");
            exit(-1);
    }
}

void switch_case_UCS(int* initial_state_S, int* final_state_G1, int* final_state_G2)
{
    int final_state_found_ID;
    double process_time;
    struct Node* p_final_state_node;
    struct timeval start, end;

    printf("\nUniform Cost Search Algorithm Calculation Begins\n");
    printf("                     *\n""                     *\n""                     *\n");

    gettimeofday(&start, NULL);
    p_final_state_node = Uniform_Cost_Search_Algorithm(initial_state_S, final_state_G1, final_state_G2
                                                       ,&final_state_found_ID);
    gettimeofday(&end,NULL);
    process_time = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec)*1.0E-9;

    if(p_final_state_node != NULL){
        UCS_OR_A_STAR_solution_handler(p_final_state_node
                                       ,final_state_found_ID, process_time);
    }
    else{ /* no final state found */
        return;
    }
}

void switch_case_A_STAR(int* initial_state_S, int* final_state_G1, int* final_state_G2)
{
    int final_state_found_ID;
    double process_time;
    struct Node* p_final_state_node;
    struct timeval start, end;

    printf("\nA_STAR Cost Search Algorithm Calculation Begins\n");
    printf("                     *\n""                     *\n""                     *\n");

    gettimeofday(&start, NULL);
    p_final_state_node = A_STAR_algorithm(initial_state_S, final_state_G1, final_state_G2
            ,&final_state_found_ID);
    gettimeofday(&end,NULL);

    process_time = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec)*1.0E-9;
    if(p_final_state_node != NULL){
        UCS_OR_A_STAR_solution_handler(p_final_state_node
                ,final_state_found_ID, process_time);
    }
}

void switch_case_BOTH(int* initial_state_S, int* final_state_G1, int* final_state_G2)
{
    switch_case_UCS(initial_state_S,final_state_G1,final_state_G2);
    g_number_of_paths_to_nodes_created = 0;


    free_linked_list(&gp_search_frontier_head, &g_search_frontier_nodes_count);
    free_linked_list(&gp_closed_set_head, &g_closed_set_nodes_count);
    reset_distances_grid();


    printf("\n\n------------------------------------------------------------------------------\n"
           "------------------------------------------------------------------------------\n"
           "------------------------------------------------------------------------------\n"
           "------------------------------------------------------------------------------\n"
           "------------------------------------------------------------------------------\n"
           "------------------------------------------------------------------------------\n\n\n");
    switch_case_A_STAR(initial_state_S,final_state_G1,final_state_G2);
}

struct Node* Uniform_Cost_Search_Algorithm(int* initial_state_S, int* final_state_G1, int* final_state_G2
        ,int* final_state_found_ID)
{
    if(initial_state_is_goal_state(initial_state_S, final_state_G1, final_state_G2)){
        printf("Initial State is one of the goal States\n");
        printf("No algorithm Execution");
        return NULL;
    }


    struct State* p_start_state = &gp_grid[initial_state_S[0]][initial_state_S[1]];
    create_and_add_node_to_a_set(&gp_closed_set_head,p_start_state
                                 ,&g_closed_set_nodes_count,initial_state_S[0]
                                 , initial_state_S[1], NULL);

    add_children_to_search_frontier(initial_state_S[0], initial_state_S[1], gp_closed_set_head,
                                    final_state_G1, final_state_G2, UCS);

    while (g_search_frontier_nodes_count > 0){
        struct Node* p_node_with_minimum_distance = get_node_with_minimum_distance_state_in_search_frontier();

        if(is_node_with_goal_state(p_node_with_minimum_distance, final_state_G1, final_state_G2
                                   ,final_state_found_ID)){
            return p_node_with_minimum_distance;
        }

        add_Node_in_closed_set(p_node_with_minimum_distance);
        add_children_to_search_frontier(p_node_with_minimum_distance->i, p_node_with_minimum_distance->j,
                                        p_node_with_minimum_distance, final_state_G1, final_state_G2
                                        ,UCS);
    }


    printf("Uniform Cost Search Algorithm Finished\nSearch frontier list got empty before a solution Path found\n");
    printf("No free path from initial p_state to some of final states\nReduce probability P! OR "
           "check if both of the final states are blocked states\n");
    return NULL;
}

struct Node * A_STAR_algorithm(int* initial_state_S, int* final_state_G1, int* final_state_G2,int* final_state_found_ID)
{
    if(initial_state_is_goal_state(initial_state_S, final_state_G1, final_state_G2)){
        printf("Initial State is one of the goal States\n");
        printf("No algorithm Execution");
        return NULL;
    }


    struct State * p_start_state = &gp_grid[initial_state_S[0]][initial_state_S[1]];
    create_and_add_node_to_a_set(&gp_closed_set_head, p_start_state
                                 ,&g_closed_set_nodes_count,initial_state_S[0]
                                 ,initial_state_S[1], NULL);
    add_children_to_search_frontier(initial_state_S[0], initial_state_S[1], gp_closed_set_head,
                                    final_state_G1, final_state_G2, A_STAR);


    while (g_search_frontier_nodes_count > 0){
        struct Node* p_node_with_minimum_distance = get_node_with_minimum_distance_state_in_search_frontier();
        if(is_node_with_goal_state(p_node_with_minimum_distance, final_state_G1, final_state_G2
                                   ,final_state_found_ID)){
            return p_node_with_minimum_distance;
        }

        add_Node_in_closed_set(p_node_with_minimum_distance);
        add_children_to_search_frontier(p_node_with_minimum_distance->i, p_node_with_minimum_distance->j,
                                        p_node_with_minimum_distance, final_state_G1, final_state_G2
                                        ,A_STAR);
    }


    printf("A_START Algorithm Finished\nSearch frontier list got empty before a solution Path found\n");
    printf("No free path from initial p_state to some of final states\nReduce probability P! OR "
           "check if both of the final states are blocked states\n");
    return NULL;
}

void UCS_OR_A_STAR_solution_handler(struct Node* p_final_state_node, int final_state_ID, double process_time)
{
    printf("\n\nFinal State G %d found as the shortest path solution\n",final_state_ID);
    printf("Printing solution path going throw the root ...\n\n\n");

    struct Node* p_current_node = p_final_state_node;
    double cost_of_final_state_path = p_current_node->p_state->distance_from_initial_state;

    while(p_current_node != NULL) {
        printf("----------------\n");
        printf("State Coordinates -> (%d,%d)\n", p_current_node->i, p_current_node->j);
        printf("State Value -> %d\n", p_current_node->p_state->value);
        printf("State Distance from initial state -> %lf\n", p_current_node->p_state->distance_from_initial_state);
        printf("----------------\n\n");

        p_current_node = p_current_node->p_predecessor;
    }
    printf("Cost form initial state to final p_state G_%d is %lf\n",final_state_ID,cost_of_final_state_path);
    printf("Number of paths to states created in order to find shortest path %d\n", g_number_of_paths_to_nodes_created);
    printf("Number of states visited (Closed Set length) -> %d\n", g_closed_set_nodes_count);
    printf("Process Time of the algorithm -> %.15lf seconds\n\n",process_time);
}

int state_is_in_search_frontier(struct State* p_state_to_check)
{
    struct Node* p_node = gp_search_frontier_head;
    while(p_node != NULL)
    {
        if(p_node->p_state == p_state_to_check) {
            return true;
        }
        p_node = p_node->p_next;
    }
    return false;
}

int is_node_with_goal_state(struct Node * p_node_to_check, const int* final_state_G1, const int* final_state_G2
        , int* final_state_ID)
{
    int state_grid_position[2] = {p_node_to_check->i, p_node_to_check->j};

    if(state_grid_position[0] == final_state_G1[0] && state_grid_position[1] == final_state_G1[1]){
        *(final_state_ID) = 1;
        return true;
    }
    else if(state_grid_position[0] == final_state_G2[0] && state_grid_position[1] == final_state_G2[1]){
        *(final_state_ID) = 2;
        return true;
    }
    else{
        return false;
    }
}

struct Node* get_node_with_minimum_distance_state_in_search_frontier()
{
    struct Node* p_current_node = gp_search_frontier_head;

    /* Case list has one node */
    if(g_search_frontier_nodes_count == 1){
        gp_search_frontier_head = NULL;
        g_search_frontier_nodes_count--;
        return p_current_node;
    }

    struct Node* p_node_with_minimum_distance_state = gp_search_frontier_head;
    struct Node* p_previous_node_from_deleted_node= NULL;
    struct Node* p_previous_node = gp_search_frontier_head;

    p_current_node = p_current_node->p_next;
    while (p_current_node != NULL){

        if(p_current_node->p_state->distance_from_initial_state <
                p_node_with_minimum_distance_state->p_state->distance_from_initial_state)
        {
            p_node_with_minimum_distance_state = p_current_node;
            p_previous_node_from_deleted_node = p_previous_node;
        }
        p_previous_node = p_previous_node->p_next;
        p_current_node = p_current_node->p_next;
    }

    /* Case when first node is one to be deleted */
    if(p_previous_node_from_deleted_node == NULL){
        gp_search_frontier_head = gp_search_frontier_head->p_next;
    }
    else{
        p_previous_node_from_deleted_node->p_next = p_node_with_minimum_distance_state->p_next;
    }

    p_node_with_minimum_distance_state->p_next = NULL;
    g_search_frontier_nodes_count--;
    return p_node_with_minimum_distance_state;
}

void add_children_to_search_frontier(int i, int j, struct Node* p_predecessor, int* final_state_G1, int* final_state_G2
        , int algorithm_ID)
{
    struct State* p_current_state = &gp_grid[i][j];

    if(j + 1 < N){
        struct State* p_neighbor_state_1 = &gp_grid[i][j + 1];

        if(p_neighbor_state_1->is_free && !state_is_in_closed_set(p_neighbor_state_1)
           && !state_is_in_search_frontier(p_neighbor_state_1)){

            p_neighbor_state_1->distance_from_initial_state =
                    abs(p_current_state->value - p_neighbor_state_1->value)
                    + 1 + p_current_state->distance_from_initial_state
                    +(algorithm_ID ? heuristic_function(i,j,final_state_G1,final_state_G2) : 0 );

            create_and_add_node_to_a_set(&gp_search_frontier_head, p_neighbor_state_1,
                                         &g_search_frontier_nodes_count, i, j + 1, p_predecessor);
            g_number_of_paths_to_nodes_created++;
        }
    }




    if(j - 1 >=0){
        struct State* p_neighbor_state_2 = &gp_grid[i][j - 1];

        if(p_neighbor_state_2->is_free && !state_is_in_closed_set(p_neighbor_state_2)
           && !state_is_in_search_frontier(p_neighbor_state_2)){

            p_neighbor_state_2->distance_from_initial_state =
                    abs(p_current_state->value - p_neighbor_state_2->value)
                    + 1 + p_current_state->distance_from_initial_state
                    +(algorithm_ID ? heuristic_function(i,j,final_state_G1,final_state_G2) : 0 );


            create_and_add_node_to_a_set(&gp_search_frontier_head, p_neighbor_state_2,
                                         &g_search_frontier_nodes_count, i, j - 1, p_predecessor);
            g_number_of_paths_to_nodes_created++;
        }
    }




    if(i+1 < N){
        struct State* p_neighbor_state_3 = &gp_grid[i + 1][j];

        if(p_neighbor_state_3->is_free && !state_is_in_closed_set(p_neighbor_state_3)
           && !state_is_in_search_frontier(p_neighbor_state_3)){

            p_neighbor_state_3->distance_from_initial_state =
                    abs(p_current_state->value - p_neighbor_state_3->value)
                    + 1 + p_current_state->distance_from_initial_state
                    +(algorithm_ID ? heuristic_function(i,j,final_state_G1,final_state_G2) : 0 );


            create_and_add_node_to_a_set(&gp_search_frontier_head, p_neighbor_state_3,
                                         &g_search_frontier_nodes_count, i + 1, j, p_predecessor);
            g_number_of_paths_to_nodes_created++;
        }

        if(j+1 < N){
            struct State* p_neighbor_state_4 = &gp_grid[i + 1][j + 1];

            if(p_neighbor_state_4->is_free && !state_is_in_closed_set(p_neighbor_state_4)
               && !state_is_in_search_frontier(p_neighbor_state_4)){

                p_neighbor_state_4->distance_from_initial_state =
                        abs(p_current_state->value - p_neighbor_state_4->value)
                        + 0.5 + p_current_state->distance_from_initial_state
                        +(algorithm_ID ? heuristic_function(i,j,final_state_G1,final_state_G2) : 0 );

                create_and_add_node_to_a_set(&gp_search_frontier_head, p_neighbor_state_4,
                                             &g_search_frontier_nodes_count, i + 1, j + 1, p_predecessor);
                g_number_of_paths_to_nodes_created++;
            }
        }

        if(j-1 >=0){
            struct State* p_neighbor_state_5 = &gp_grid[i + 1][j - 1];

            if(p_neighbor_state_5->is_free && !state_is_in_closed_set(p_neighbor_state_5)
               && !state_is_in_search_frontier(p_neighbor_state_5)){

                p_neighbor_state_5->distance_from_initial_state =
                        abs(p_current_state->value - p_neighbor_state_5->value)
                        + 0.5 + p_current_state->distance_from_initial_state
                        +(algorithm_ID ? heuristic_function(i,j,final_state_G1,final_state_G2) : 0 );

                create_and_add_node_to_a_set(&gp_search_frontier_head, p_neighbor_state_5,
                                             &g_search_frontier_nodes_count, i + 1, j - 1, p_predecessor);
                g_number_of_paths_to_nodes_created++;
            }
        }
    }



    if(i-1 >= 0){
        struct State* p_neighbor_state_6 = &gp_grid[i - 1][j];

        if(p_neighbor_state_6->is_free && !state_is_in_closed_set(p_neighbor_state_6)
           && !state_is_in_search_frontier(p_neighbor_state_6)){

            p_neighbor_state_6->distance_from_initial_state =
                    abs(p_current_state->value - p_neighbor_state_6->value)
                    + 1 + p_current_state->distance_from_initial_state
                    +(algorithm_ID ? heuristic_function(i,j,final_state_G1,final_state_G2) : 0 );

            create_and_add_node_to_a_set(&gp_search_frontier_head, p_neighbor_state_6,
                                         &g_search_frontier_nodes_count, i - 1, j, p_predecessor);
            g_number_of_paths_to_nodes_created++;
        }

        if(j+1 < N ){
            struct State* p_neighbor_state_7 = &gp_grid[i - 1][j + 1];

            if(p_neighbor_state_7->is_free && !state_is_in_closed_set(p_neighbor_state_7)
               && !state_is_in_search_frontier(p_neighbor_state_7)){

                p_neighbor_state_7->distance_from_initial_state =
                        abs(p_current_state->value - p_neighbor_state_7->value)
                        + 0.5 + p_current_state->distance_from_initial_state
                        +(algorithm_ID ? heuristic_function(i,j,final_state_G1,final_state_G2) : 0 );

                create_and_add_node_to_a_set(&gp_search_frontier_head, p_neighbor_state_7,
                                             &g_search_frontier_nodes_count, i - 1, j + 1, p_predecessor);
                g_number_of_paths_to_nodes_created++;
            }
        }

        if(j-1 >=0){
            struct State* p_neighbor_state_8 = &gp_grid[i - 1][j - 1];

            if(p_neighbor_state_8->is_free && !state_is_in_closed_set(p_neighbor_state_8)
               && !state_is_in_search_frontier(p_neighbor_state_8)){

                p_neighbor_state_8->distance_from_initial_state =
                        abs(p_current_state->value - p_neighbor_state_8->value)
                        + 0.5 + p_current_state->distance_from_initial_state
                        +(algorithm_ID ? heuristic_function(i,j,final_state_G1,final_state_G2) : 0) ;

                create_and_add_node_to_a_set(&gp_search_frontier_head, p_neighbor_state_8,
                                             &g_search_frontier_nodes_count, i - 1, j - 1, p_predecessor);
                g_number_of_paths_to_nodes_created++;
            }
        }
    }
}

void create_and_add_node_to_a_set(struct Node ** set_head, struct State* p_state, int* nodes_count, int i, int j
        , struct Node* p_predecessor)
{
    struct Node* p_new_node = (struct Node *) malloc(1 * sizeof (struct Node));
    p_new_node->i = i;
    p_new_node->j = j;
    p_new_node->p_state = p_state;
    p_new_node->p_predecessor = p_predecessor;
    p_new_node->p_next = (*nodes_count < 1) ? NULL : *set_head;
    *set_head = p_new_node;
    (*nodes_count)++;
}

int state_is_in_closed_set(struct State* p_state_to_check)
{
    struct Node* p_node = gp_closed_set_head;
    while(p_node != NULL){

        if(p_node->p_state == p_state_to_check){
            return true;
        }
        p_node = p_node->p_next;
    }
    return false;
}

void add_Node_in_closed_set(struct Node* p_node)
{
    if(g_closed_set_nodes_count++ < 1){
        gp_closed_set_head = p_node;
    }
    else{
        p_node->p_next = gp_closed_set_head;
        gp_closed_set_head = p_node;
    }
}

int initial_state_is_goal_state(const int* current_state, const int* final_state_G1, const int* final_state_G2)
{
    return ((current_state[0] == final_state_G1[0] && current_state[1] == final_state_G1[1]) ||
            (current_state[0] == final_state_G2[0] && current_state[1] == final_state_G2[1]));
}

void free_linked_list(struct Node** head_address,int* nodes_count)
{
    struct Node* p_temp , *p_head , **pp_keep_old_head ;
    p_head = *head_address;
    pp_keep_old_head = head_address;
    while (p_head != NULL){
        p_temp = p_head;
        p_head = p_head->p_next;
        free(p_temp);
    }

    *(nodes_count) = 0;
    *pp_keep_old_head = NULL;
}

void free_states(struct State** grid)
{
    for(int i=0; i<N; i++)
        free(grid[i]);
    free(grid);

    gp_grid = NULL;
}

double heuristic_function(int i, int j, int* final_state_G1, int* final_state_G2)
{
    double dx_1, dy_1 , dx_2 , dy_2;

    dx_1 = abs(final_state_G1[0] - i);
    dx_2 = abs(final_state_G2[0] - i);

    dy_1 = abs(final_state_G1[1] - j);
    dy_2 = abs(final_state_G2[1] - j);

    return min(2*(dx_1+dy_1) + (sqrt(2) - 2)*min(dx_1,dy_1)
               , 2*(dx_2+dy_2) + (sqrt(2) - 2)*min(dx_2,dy_2));

}

double min(double num_1, double num_2)
{
    return num_1 < num_2 ? num_1 : num_2;
}

void print_nodes_in_search_frontier(void)
{
    struct Node * p_node = gp_search_frontier_head;
    printf("Nodes in search Frontier:\n\n");
    int count = 1;
    while(p_node != NULL){
        printf("Node Count : %d\n",count++);
        printf("State position in gp_grid -> (%d,%d)\n", p_node->i, p_node->j);
        printf("State Value : %d\n", p_node->p_state->value);
        printf("State Distance from initial State : %lf\n", p_node->p_state->distance_from_initial_state);
        p_node = p_node->p_next;
        printf("-------------------------------------------------------\n");
    }
}

void print_nodes_in_closed_set(void)
{
    struct Node * p_node = gp_closed_set_head;
    printf("Closed Set Nodes :\n");
    int count = 1;
    while(p_node != NULL){

        printf("Node %d\n",count++);
        printf("State Position in gp_grid ->(%d,%d)\n", p_node->i, p_node->j);
        printf("State value: %d\n", p_node->p_state->value);
        printf("State distance: %lf\n", p_node->p_state->distance_from_initial_state);
        printf("--------------------\n");
        p_node = p_node->p_next;
    }
}

