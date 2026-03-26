// net_bot.h — bot network driver
// Provides fake network connections for bots.
// Register in net_drivers[] to enable.

int			Bot_Init (void);
void		Bot_Listen (qboolean state);
void		Bot_SearchForHosts (qboolean xmit);
qsocket_t	*Bot_Connect (char *host);
qsocket_t	*Bot_CheckNewConnections (void);
int			Bot_GetMessage (qsocket_t *sock);
int			Bot_SendMessage (qsocket_t *sock, sizebuf_t *data);
int			Bot_SendUnreliableMessage (qsocket_t *sock, sizebuf_t *data);
qboolean	Bot_CanSendMessage (qsocket_t *sock);
qboolean	Bot_CanSendUnreliableMessage (qsocket_t *sock);
void		Bot_Close (qsocket_t *sock);
void		Bot_Shutdown (void);
