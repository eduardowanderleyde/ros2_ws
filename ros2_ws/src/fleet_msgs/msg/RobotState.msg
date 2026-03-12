# Estado agregado de um robô para /fleet/status (UI)
string robot_id
string role           # MUUT | FUUT | SU (papel no experimento)
string nav_state      # idle | recording | navigating | failed
string current_route  # nome da rota em execução ou vazio
bool collection_on
string collection_file
string last_error     # último erro (TF, Nav2, etc.) ou vazio
uint64 bytes_written  # bytes no bag atual (0 se não coletando)
