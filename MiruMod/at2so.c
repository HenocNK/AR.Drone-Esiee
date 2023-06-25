/*
 * what: at2so - drone command relay (proxy)
 * who:  miru
 * when: April 2011...
 *
 * ARMTC = /home/arm-2010.09/bin/arm-none-linux-gnueabi
 * ARMCF =
 * ARMCF += -Wall
 * ARMCF += -Wstrict-prototypes
 * #ARMCF += -mfloat-abi=softfp	# NOT FOR DRONE 1
 * ARMCF += -Os
 * 
 * %.arm: %.c
 *	$(ARMTC)-gcc $(ARMCF) $< -o $@ -lm -lrt
 *	$(ARMTC)-strip $@
 */
char version[] = "@(#) at2so 0.23 20141110";

#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#include <time.h>
typedef struct timeval	tmv_t;
typedef struct timespec	tms_t;

#include <termios.h>
typedef struct termios	tio_t;

#include <netdb.h>
#include <netinet/in.h>
typedef union {
	struct sockaddr     sa;
	struct sockaddr_in  sin;
}	soa_t;
#define	sin_fmly	sin.sin_family
#define	sin_port	sin.sin_port
#define	sin_addr	sin.sin_addr.s_addr
#define	IPV4A(a,b,c,d)	htonl(((a&0xff)<<24)|((b&0xff)<<16)|((c&0xff)<<8)|(d&0xff))

typedef union { int i; float f; } vif_t;
typedef unsigned char	u08_t;
typedef unsigned short	u16_t;
typedef unsigned int	u32_t;

#define	NEL(x)		(int)(sizeof(x)/sizeof(x[0]))
#define	MAX(a,b)	((a)>(b)?(a):(b))
#define	ABSMAX(x,v)	((v)<-(x)?-(x):(v)>(x)?(x):(v))
#define	MINMAX(n,x,v)	((v)<(n)?(n):(v)>(x)?(x):(v))
#define	DEG2RAD(d)	((double)(d)*M_PI/180.0)
#define	RAD2DEG(r)	((double)(r)*180.0/M_PI)

#define	gps_in(b)	BLOG(b)
#define	bmp_in(b)	BLOG(b)
#define	hmc_in(b)	BLOG(b)
#define	m56_in(b)	BLOG(b)

/*
 * drone firmware particulars ...
 */
typedef struct { /* navdata header */
	u32_t	mag;
#define	NAVMAG0	0x55667788	/* full current data set */
#define	NAVMAG1	0x55667789	/* intermediate */
	u32_t	cst;
	/* according to SDK 2.1 */
#define	AS_EMERGENCY	(1<<31) /* 1 emergency landing */
#define	AS_COM_WATCHDOG	(1<<30) /* 1 communication watchdog trigger */
#define	AS_ADC_WATCHDOG	(1<<29) /* NA 1 delay in uart2 dsr (>5ms) */
#define	AS_CTL_WATCHDOG	(1<<28) /* NA 1 delay in control execution */
#define	AS_ACQ_THREAD	(1<<27) /* NA 1 acquisition thread on */
#define	AS_VID_THREAD	(1<<26) /* NA 1 video thread on */
#define	AS_NAV_THREAD	(1<<25) /* NA 1 navdata thread on */
#define	AS_AT__THREAD	(1<<24) /* NA 1 AT codec thread on */
#define	AS_PIC_V_OK	(1<<23) /* NA 1 PIC version ok */*/
#define	AS_CUTOUT	(1<<22) /* NA 1 motor emergency stop */
#define	AS_ULTRASOUND	(1<<21) /* 1 ultrasound sensor problem */
#define	AS_WIND		(1<<20) /* NA 1 too much wind */
#define	AS_ANGLES_OOR	(1<<19) /* NA 1 angles out of range */
#define	AS_MAG_CALRQ	(1<<18) /* NA 1 magnetometer needs calibration */
#define	AS_TIMER	(1<<17) /* NA 1 timer elapsed */
#define	AS_USER_ESTOP	(1<<16) /* NA 1 user emergency stop */
#define	AS_VBAT_LOW	(1<<15) /* 1 battery too low to fly */
#define	AS_SW_FAULT	(1<<14) /* NA 1 software fault */
#define	AS_COM_LOST	(1<<13) /* NA 1 communication problem */
#define	AS_MOTORS	(1<<12) /* NA 1 motor problem */
#define	AS_NAV_BOOT	(1<<11) /* NA 1 no navdata options */
#define	AS_NAV_DEMO	(1<<10) /* NA 1 only navdata demo */
#define	AS_USB		(1<< 9) /* NA 1 USB key ready */
#define	AS_TRAVELLING	(1<< 8) /* NA 1 travelling enable ? */
#define	AS_CAMERA	(1<< 7) /* NA 1 camera ready */
#define	AS_COMMAND_ACK	(1<< 6) /* 1 command ack */
#define	AS_USER_STRT	(1<< 5) /* NA 1 user start button on */
#define	AS_ALTCTRL	(1<< 4) /* 1 altitude control active */
#define	AS_ANGCTRL	(1<< 3) /* NA 1 angular speed control, 0 euler angle control */
#define	AS_VISION	(1<< 2) /* NA 1 vision enable */
#define	AS_VIDEO	(1<< 1) /* NA 1 video enable */
#define	AS_FLY		(1<< 0) /* 1 flying */
	u32_t	seq;
	u32_t	vdf;
}	__attribute__((packed)) navh_t;

typedef struct { /* navdata option header */
	short	tag;
	u16_t	siz;
}	__attribute__((packed)) noph_t;

typedef struct { /* tag -1 checksum */
	noph_t	hdr;
	u32_t	cks;
}	__attribute__((packed)) ntck_t;

typedef struct { /* tag 00 */
	noph_t	hdr;
	u32_t	stat;	/* ctrl_state */
	u32_t	cbat;	/* vbat_flying_percentage (23 bits) */
	float	theta;
	float	phi;
	float	psi;
	int	altitude;
	struct {
		float	vx;
		float	vy;
		float	vz;
		u32_t	num_frames;
		float	detection_camera[12];	/* SDK 1.5 calls it obsolete */
		u32_t	detection_tag_index;	/* SDK 1.5 calls it obsolete */
		int	detection_camera_type;
		float	drone_camera[12];	/* SDK 1.6 calls it obsolete */
	}	trash;
}	__attribute__((packed)) nt00_t;

typedef struct { /* tag 25 (navdata_hdvideo_stream_t) */
	noph_t	hdr;
	u32_t	hdvideo_state;
#define	HDVIDEO_STORAGE_FIFO_IS_FULL (1<< 0)
#define	HDVIDEO_USBKEY_IS_PRESENT    (1<< 8)
#define	HDVIDEO_USBKEY_IS_RECORDING  (1<< 9)
#define	HDVIDEO_USBKEY_IS_FULL       (1<<10)
	u32_t	storage_fifo_nb_packets;
	u32_t	storage_fifo_size;
	u32_t	usbkey_size;
	u32_t	usbkey_freespace;
	u32_t	frame_number;
	u32_t	usbkey_remaining_time;	/* in seconds */
}	__attribute__((packed)) nt25_t;

typedef struct { /* tag 27 (navdata_gps_t) */
	noph_t	hdr;
	double	lat;
	double	lon;
	double	alt;
	double	hdop;
	u32_t	stat[3];
	double	lat0;
	double	lon0;
	double	lat1;
	double	lon1;
	u32_t	gps_state;
	u08_t	unk1[56];
	float	sog;	/* GPS speed over ground  [m/s * 10^2] */
	u32_t	lfts;	/* last frame timestamp */
	float	cog;	/* GPS course over ground [deg * 10^2] */
	float	funk;
	float	ehpe;	/* GPS EHPE [m   * 10^2] */
	float	ehve;	/* GPS EHVE [m/s * 10^2] */
	float	eunk;
	u32_t	nsat;
	struct { u08_t	sat, cn0; } channels[12];
	int	gps_plugged;
	u08_t	unk3[16];	/* [280] on 2.4.7 */
}	__attribute__ ((packed)) nt27_t; /* (216/480 bytes) */

typedef enum {
/*  0 */	BLINK_GREEN_RED,
/*  1 */	BLINK_GREEN,
/*  2 */	BLINK_RED,
/*  3 */	BLINK_ORANGE,
/*  4 */	SNAKE_GREEN_RED,
/*  5 */	FIRE,
/*  6 */	STANDARD,
/*  7 */	RED,
/*  8 */	GREEN,
/*  9 */	RED_SNAKE,
/* 10 */	BLANK,
/* 11 */	RIGHT_MISSILE,
/* 12 */	LEFT_MISSILE,
/* 13 */	DOUBLE_MISSILE,
/* 14 */	FRONT_LEFT_GREEN_OTHERS_RED,
/* 15 */	FRONT_RIGHT_GREEN_OTHERS_RED,
/* 16 */	REAR_RIGHT_GREEN_OTHERS_RED,
/* 17 */	REAR_LEFT_GREEN_OTHERS_RED,
/* 18 */	LEFT_GREEN_RIGHT_RED,
/* 19 */	LEFT_RED_RIGHT_GREEN,
/* 20 */	BLINK_STANDARD,
}	led_t;

static int strequ(char *s, char *d)
{
	while (1) if (*s != *d++) break; else if (*s++ == 0) return 1;
	return 0;
}

static char *strbeg(char *hdr, char *str)
{
	while (*hdr) if (*hdr == *str) hdr++, str++; else return 0;
	return str;
}

static char *strtxf(char *s, char *d, int n)
{
	while (--n >= 0 && (*d = *s++)) d++;
	return d;
}

static char *hexstr(void *p, int n, char *d)
{
	u08_t	*b = p;

	while (--n >= 0) d += sprintf(d,"%02X",*b++);
	return d;
}

static int cmd2av(char *s, char **av, int ma)
{
	int	n = 0, i;

	for (av[n++] = s; *s && *s != '=' && *s != ','; s++);
	if (*s)
		for (i = 0, *s++ = 0, av[n++] = s; *s && n < ma; s++)
			if (*s == '\"') i ^= 01;
			else if (!i && *s == ',') *s = 0, av[n++] = s + 1;
	/* take out quotes */
	for (i = 0; i < n; i++)
		if (*(s = av[i]) == '\"') {
			for (av[i] = ++s; *s; s++);
			if (*(--s) == '\"') *s = 0;
		}
	return n;
}

/* configuration mechanism */
typedef struct {
	char	var[32];
	char	val[40];
}	cvv_t;

typedef struct {
	int	cvvr;
	int	cvvw;
	cvv_t	cvvb[32];
}	cfg_t;

/* command cell */
typedef struct {
	int	alin;	/* alignment */
	int	mode;
	vif_t	rpgy[4];
}	pcmd_t;

/*
 * global variables
 */
struct {
	int	exit;
	tms_t	tref;	/* reference time for gl.tnow */
	u32_t	tnow;
	u32_t	flag;	/* program flags */
#define	F_BLK_WIFI	0x00000001

	soa_t	dcmd; /* drone cmd port on network */
	soa_t	dnav; /* drone nav port on network */
	soa_t	ddat; /* drone dat port on network */

	int	md; fd_set sd;	/* file desciptors to watch */
	int	cd; soa_t ca;	/* cmd socket, sends to port 5556 */
	int	id; soa_t ia;	/* proxy dat socket, outside gets redirected here */
	int	fd; soa_t fa;	/* proxy dat sender socket (transient) */
	int	dd; soa_t da;	/* proxy dat receiver, drone is told to send here */
	int	nd; soa_t na;	/* proxy nav socket, outside gets redirected here */
	int	rd; soa_t ra;	/* proxy nav receiver, drone is told to send here */
	int	ad; tio_t ap[2];/* Arduino serial port */
	int	ld;		/* log file desciptor */

	struct ardu {
		u32_t	trcv;	/* tnow when last message was received */
		u32_t	tout;	/* sequencer timeout */
		int	swdg;	/* status watchdog sequencer */
		int	scfg;	/* status configuration sequencer */
		int	rcfg;	/* configuration sequencer retry count */
		int	acfg;	/* Arduino wants ACK when configs are done */
		u32_t	sdto;	/* Arduino data transmission timeout */
		int	link;	/* serial link status */
		pcmd_t	ucmd; 	/* user steering commands */
		u32_t	disc;	/* tnow on TX disconnect */
		u32_t	reco;	/* tnow on TX reconnect */
		int	dseq;	/* disconnect sequencer */
		u32_t	dtot;	/* disconnect timeout */
		pcmd_t	dcmd; 	/* disconnect commands */
	}	ardu;

	struct idev {
		u32_t	trcv;	/* tnow when last message was received */
		soa_t	fnav;	/* forwarding address for navdata */
		int	nall;	/* forward all Navdata, header only otherwise */
		u32_t	fcst;	/* cst bits for iDev */
#define	IDEVMCST	(AS_COM_WATCHDOG|AS_COMMAND_ACK)
	}	idev;

	/* drone */
	u32_t	seq;	/* AT command sequence number */
	int	nat;	/* AT command buffer fill (at_send()) */
	char	atb[512];

	char	dfmw[16]; /* drone firmware version */
	int	ard2;	/* is AR.drone 2.0 */
	int	anim;	/* animation enable */
	int	dcat;	/* disconnect action deferral */
	int	vzap;	/* current video zap */
	int	rvid;	/* D2 record flight video on USB stick */
	int	airb;	/* is airborne */
	int	estp;	/* emergency stop sequencer */

	navh_t	navh;	/* last navdata header */
	u32_t	nnav;	/* number of navdata packets received */
	u32_t	tnav;	/* timestamp of last navdata receipt */
	struct {	/* last tag 0 extracts */
		int	bat;	/* battery percentage */
		int	psi;	/* heading in [deg * 1000] */
		int	alt;	/* altitude in D2 [mm], D1 [?] */
	}	nt00;
	struct {	/* last tag 25 extracts */
		int	rvok;	/* record video to USB stick ok */
	}	nt25;

	cfg_t	cfgs;	/* configuration command queue */

	struct smgr {	/* session management */
		char	*sid;	/* session id */
		char	*pid;	/* profile id */
		char	*aid;	/* application id */

		int	cex;	/* data type to expect from tcp:5559 */
		int	c6n;	/* configuration choices CTRL,6 */
		char	c6b[1*1024];
		int	c4n;	/* configuration variables CTRL,4 */
		char	c4b[7*1024];
	}	smgr;

	int	lgi;	/* log index for log file name */
	int	lgs;	/* current size of log file */
#define	MLGS	(4*1024*1024)
	char	lgb[512];	/* slog temporary buffer */
}	gl;

static int add_fd(int fd)
{
	if (fd >= 0) {
		FD_SET(fd,&gl.sd);
		if (fd >= gl.md) gl.md = fd + 1;
	}
	return fd;
}

static int del_fd(int fd)
{
	if (fd >= 0) {
		FD_CLR(fd,&gl.sd);
		while (gl.md > 0 && !FD_ISSET(gl.md-1,&gl.sd)) gl.md--;
	}
	return -1;
}

static int add_soc(int typ, soa_t *sa, int nw)
{
	int	sd, n;
	socklen_t sl;

	sa->sin_fmly = AF_INET;
	if ((sd = socket(sa->sin_fmly,typ,0)) < 0) return -1;
	while (1) {
		if (ioctl(sd,FIONBIO,&nw)) break;
		sl = sizeof(sa->sa);
		if (bind(sd,&sa->sa,sl)) break;
		if (getsockname(sd,&sa->sa,&sl)) break;
		return add_fd(sd);
	}
	n = errno; close(sd); errno = n;
	return -1;
}

static int del_soc(int sd)
{
	if (sd >= 0) close(sd);
	return del_fd(sd);
}

/* Linux time is an 8 byte entity which is a little large for here, using 'tics'
 * since start of program at a lower resolution to make it fit into 32 bit variable.
 * A u32_t with 1 ms resolution rolls after ~49.7 days	*/
#define	NTPS		1000
#define	S2TIC(s)	((int)(((double)(s))*NTPS+0.5))

#define	CLOCK_SEL	CLOCK_MONOTONIC

static int tref(void)
{
	return clock_gettime(CLOCK_SEL,&gl.tref);
}

static u32_t tnow(void)
{
	tms_t	tv;
	u32_t	tnow;

	clock_gettime(CLOCK_SEL,&tv);
#define	NSUBSEC 1000000000L
	tv.tv_sec -= gl.tref.tv_sec;
	tnow = tv.tv_sec * NTPS + tv.tv_nsec / (NSUBSEC/NTPS);
	return tnow;
}

static void slog_b(int n)
{
	int	k;
	char	buf[32+128];

	if (gl.ld < 0 || n < 0) return;
	if (n > 128) n = 128;
	k = write(gl.ld,buf,sprintf(buf,"%7u: %.*s\n",tnow(),n,gl.lgb));
	if (k <= 0 || (gl.lgs += k) >= MLGS) close(gl.ld), gl.ld = -2;
}

#define	SLOG(...)	slog_b(sprintf(gl.lgb,__VA_ARGS__))
#if !defined(FLASH)
#define	BLOG(...)	slog_b(sprintf(gl.lgb,__VA_ARGS__))
#else
#define	BLOG(...)
#endif

/*
 * dealing with ... session mechanism for iDev
 */
static void smgr_c4bi(struct smgr *sm)
{
	char	*s, *d, *e;

	if (sm->c4n <= 0 || sm->c4n >= NEL(sm->c4b)) sm->c4n = 0;
	for (e = (s = d = sm->c4b) + sm->c4n; s < e; s++)
		if (*s < ' ' && *s != '\n') {
			if (*s == '\t') *d++ = ' ';
			else if (*s == '\r' && s[1] != '\n') *d++ = '\n';
		}
		else if (s == d) d++; else *d++ = *s;
	if ((e = --d) < sm->c4b) e = sm->c4b;
	else if (*e == '\n') *e++ = 0;
	else if (*e)    e++, *e++ = 0;
	sm->c4n = e - sm->c4b;
}

static void smgr_c4bu(struct smgr *sm, char *var, char *val)
{
	char	*s, *b, *e;
	int	l, n, k;

	if (*val == '-' && strequ("-all",val) && strequ("custom:session_id",var)) return;
	l = strlen(var) + 3 + strlen(val);
	for (e = (s = sm->c4b) + sm->c4n; s < e; ) {
		while (*s < ' ') s++;
		for (b = s; s < e && *s >= ' '; s++);
		if (!strbeg(var,b)) continue;
		if ((k = l - (s - b))) {
			if ((n = sm->c4n + k) < 0 || n > NEL(sm->c4b)) return;
			bcopy(s,s+k,e-s);
			sm->c4n += k; e += k;
		}
		b[sprintf(b,"%s = %s",var,val)] = '\n'; *(--e) = 0;
		return;
	}
	if ((sm->c4n + l) >= NEL(sm->c4b)) return;
	sm->c4n += sprintf(--e,"\n%s = %s",var,val);
}

/*
 * Note: I can't really see a difference if I am doing this so it might go away...
 */
#if	0
#define	smgr_c4bf(s,t,i)
#else
static void smgr_c4bf(struct smgr *sm, char *typ, char *ids)
{
	char	*c, *s, *e, *b, *p, *t;
	int	fd, n;
	char	var[128], buf[8*4096];

	if (*ids == '-') return;
	if ((s = strrchr(typ,'_')) == 0) return;
	if (strequ("_id",s) == 0) return;
	     if (*typ == 'a') s = "applis";
	else if (*typ == 'p') s = "profiles";
	else if (*typ == 's') s = "sessions";
	else return;
	sprintf(var,"/data/custom.configs/%s/config.%.8s.ini",s,ids);
	if ((fd = open(var,0)) < 0) n = 0;
	else n = read(fd,buf,sizeof(buf)), close(fd);
	BLOG("C4BF,%d,%d,%s",fd,n,var);
	if (n <= 0) return;
	for (c = var, e = (s = buf) + n; s < e; ) {
		while (  s < e && *s <= ' ') s++; b = s;
		while (  s < e && *s >= ' ') s++;
		while (--s > b && *s == ' '); s++;
		while (  s < e && *s <= ' ') *s++ = 0;
		if (*b == '[' && (p = strrchr(b,']')))
			b++, c = var + sprintf(var,"%.*s:",p-b,b);
		else if ((p = strchr(b,'=')) == 0) continue;
		else {
			for (*(t = p) = 0; --t >= b && *t == ' '; *t = 0);
			while (++p < s && *p == ' ');
			sprintf(c,"%s",b);
			smgr_c4bu(sm,var,p);
		}
	}
}
#endif

/* configuration cmd fifo */
static void psh_cfg(char *var, char *val)
{
	cfg_t	*cfg = &gl.cfgs;
	cvv_t	*cvv;
	int	i;

	cvv = &cfg->cvvb[i = cfg->cvvw];
	bzero(cvv,sizeof(*cvv));
	strtxf(var,cvv->var,NEL(cvv->var)-1);
	strtxf(val,cvv->val,NEL(cvv->val)-1);
	if (++i >= NEL(cfg->cvvb)) i = 0;
	if (i == cfg->cvvw) return;
	cfg->cvvw = i;
}

static cvv_t *nxt_cfg(void)
{
	cfg_t	*cfg = &gl.cfgs;

	return cfg->cvvr == cfg->cvvw ? 0 : &cfg->cvvb[cfg->cvvr];
}

static int pop_cfg(void)
{
	cfg_t	*cfg = &gl.cfgs;
	int	i;

	if ((i = cfg->cvvr) != cfg->cvvw) cfg->cvvr = ++i >= NEL(cfg->cvvb) ? 0 : i;
	return 0;
}

/* AT cmd editor */
static void at_send(char *dat)
{
	char	*s, *b, *d, *p;
	int	k;

	d = gl.atb + gl.nat;
	for (s = dat; *s; s += *s ? 1 : 0) {
		for (b = s; *s && *s != '\r'; s++);		/* find end of command */
		p = d; *p++ = 'A'; *p++ = 'T'; *p++ = '*';	/* 'AT*' key prefix for drone */
		while (b < s && (*p++ = *b++) != '=');		/* copy <key>= */
		if ((k = (p - d) - 4) <= 0) continue;		/* <key> length */
		while (b < s && *b != ',') b++;			/* skip sequence number */
		if (k == 6 && d[3] == 'C' && d[5] == 'N') {	/* need CONFIG_IDS */
			p--;
			p += sprintf(p,"_IDS=%u,\"%s\",\"%s\",\"%s\"\rAT*CONFIG=",
				++gl.seq,gl.smgr.sid,gl.smgr.pid,gl.smgr.aid);
		}
		p += sprintf(p,"%u",++gl.seq);	/* add local sequence number */
		while (b < s) *p++ = *b++;	/* copy arguments 'as is' */
		*p++ = '\r';
		d = p;
	}
	gl.nat = d - gl.atb;
}

static void led(led_t typ, float fhz, int dur)
{
	vif_t	v;
	char	cmd[32];

	v.f = fhz;
	sprintf(cmd,"LED=,%d,%d,%d",typ,v.i,dur);
	at_send(cmd);
}

static void nrq_in(void)
{
	soa_t	fa;
	int	n, dat[64];
	socklen_t fl;

	fl = sizeof(fa.sa);
	if ((n = recvfrom(gl.nd,dat,sizeof(dat),0,&fa.sa,&fl)) <= 0) return;
	BLOG("NRQI,%d,%d",n,dat[0]);
	if (n != 4) return;
	switch (dat[0]) {
	case 0: gl.idev.fnav.sin_port = 0; break;	/* turn it off */
	case 1: gl.idev.fnav = fa; break;		/* navdata to specific socket request */
	}
}

static ntck_t *nav_ckt(void *dat, int len)
{
	union { u08_t *b; ntck_t *c; } p;

	if (len < (int)sizeof(*p.c)) return 0;
	p.b = dat; p.b += len; p.c--; /* checksum option is at end */
	if (p.c->hdr.siz != (u16_t)sizeof(*p.c)) return 0;
	if (p.c->hdr.tag != -1) return 0;
	return p.c;
}

#if	0
static void *nav_tag(void *dat, int len, short tag, u16_t siz)
{
	union { u08_t *b; navh_t *h; noph_t *o; } p, e;

	e.b = (p.b = dat) + len;
	for (p.h++; p.b < e.b && p.o->siz >= sizeof(*p.o); p.b += p.o->siz)
		if (p.o->tag == tag && (siz == 0 || p.o->siz == siz)) return p.o;
	return 0;
}

static int nav_del(void *dat, int len, short tag)
{
	union { u08_t *b; ntck_t *c; noph_t *o; } p, d, s;
	int	nd, k;

	if ((d.o = nav_tag(dat,len,tag,0)) == 0) return 0;
	if ((p.c = nav_ckt(dat,len)) == 0 || d.b >= p.b) return 0;
	nd = d.o->siz;
	for (s.b = d.b, k = nd; --k >= 0; s.b++) p.c->cks -= *s.b;
	for (p.c++, k = p.b - s.b; --k >= 0; *d.b++ = *s.b++);
	return nd;
}

static int nav_add(void *dat, int len, noph_t *nt)
{
	union { u08_t *b; ntck_t *c; noph_t *o; } s, p, e;

	if ((p.c = nav_ckt(dat,len)) == 0) return 0;
	if (nt->siz <= (int)sizeof(*nt) || nt->siz & 03) return 0;
	s.o = nt;
	e.b = p.b + nt->siz;
	*e.c = *p.c;
	while (p.b < e.b) e.c->cks += (*p.b++ = *s.b++);
	return nt->siz;
}
#endif

static int nav_dat(void *dat, int len, ntck_t **pck)
{
	ntck_t	*ck;
	u08_t	*b;
	u32_t	cks;

	if (len < (int)(sizeof(navh_t)+sizeof(ntck_t))) return -2;
	if ((ck = nav_ckt(dat,len)) == 0) return -3; /* no checksum option */
	for (cks = 0, b = dat; b < (u08_t *)ck; b++) cks += *b;
	if (cks != ck->cks) return -4; /* bad checksum */
	if (pck) *pck = ck;
	return 0;
}

static int nav_addcks(void *dat, int len)
{
	u08_t	*b = dat;
	ntck_t	*ntck;
	int	i;

	ntck = (ntck_t *)&b[len];
	ntck->hdr.tag = -1;
	ntck->hdr.siz = sizeof(*ntck);
	ntck->cks = 0;
	for (i = len; --i >= 0; b++) ntck->cks += *b;
	return len + ntck->hdr.siz;
}

static void nav_setcst(navh_t *nh, ntck_t *ck)
{
	u08_t	*b = (u08_t *)&nh->cst;
	int	i;

	if (ck) for (i = sizeof(nh->cst); --i >= 0; ) ck->cks -= *(b++);
	nh->cst &= ~IDEVMCST;
	nh->cst |= gl.idev.fcst;
	if (ck) for (i = sizeof(nh->cst); --i >= 0; ) ck->cks += *(--b);
}

static void nav_in(void)
{
	union { u08_t	*b;
		navh_t	*h; noph_t *o;
		nt00_t	*nt00; nt25_t *nt25; nt27_t *nt27;
	}	b, p, e;
	ntck_t	*ntck;
	u32_t	chg;
	int	n, len;
	u08_t	dat[1504];

	len = read(gl.rd,b.b=dat,sizeof(dat));
	if ((n = len <= 0 ? -1 : nav_dat(b.b,len,&ntck))) return;

	chg = gl.nnav ? gl.navh.cst : ~b.h->cst;
	gl.tnav = gl.tnow;
	gl.navh = *b.h;
	gl.nnav++;
	if ((chg ^= gl.navh.cst)) {
		if ((chg & AS_EMERGENCY)) {
			if ((gl.navh.cst & AS_EMERGENCY) && gl.estp == 0) gl.estp = 3;
			SLOG("ESTP,%d",(gl.navh.cst & AS_EMERGENCY) ? 1 : 0);
			chg &= ~AS_EMERGENCY;
		}
		if ((chg & AS_FLY)) {
			gl.airb = gl.navh.cst & AS_FLY ? 1 : 0;
			SLOG("AIRB,%d",gl.airb);
			chg &= ~AS_FLY;
		}
		if ((chg & AS_COMMAND_ACK)) {
			SLOG("CMAK,%d",gl.navh.cst & AS_COMMAND_ACK ? 1 : 0);
			chg &= ~AS_COMMAND_ACK;
		}
		if (chg) SLOG("DCST,0x%08x,0x%08x",chg,gl.navh.cst);
	}

	e.b = b.b + len;
	for (p = b, p.h++; p.b < e.b && p.o->siz >= sizeof(*p.o); p.b += p.o->siz)
		if (p.o->tag < 0) break;
		else switch (p.o->tag) {
		case 0:
			if (p.o->siz != sizeof(*p.nt00)) break;
			n = (short)p.nt00->cbat;	/* 23 bit variable [22:15] same as [14] */
			if (gl.nt00.bat != n) { gl.nt00.bat = n; SLOG("VBAT,%d",n); }
			n = (int)p.nt00->psi;
			n = (n + (n < 0 ? -500 : +500))/1000;
			if (gl.nt00.psi != n) { gl.nt00.psi = n; BLOG("HPSI,%d",n); }
			n = p.nt00->altitude;
			if (gl.nt00.alt != n) { gl.nt00.alt = n; BLOG("ALTI,%d",n); }
			break;
		case 25:
			if (p.o->siz != sizeof(*p.nt25)) break;
			gl.nt25.rvok = p.nt25->usbkey_freespace > (5*1024) && p.nt25->usbkey_size > (5*1024);
			break;
		}

	if (gl.idev.fnav.sin_port == 0) return;
#if	0
	/* this tag is large, can't be turned off and has no use for us */
	len -= nav_del(dat,len,16);
#endif
	if (((b.h->cst ^ gl.idev.fcst) & IDEVMCST)) nav_setcst(b.h,ntck);
	if (gl.idev.nall == 0) len = nav_addcks(b.b,sizeof(*b.h));
	if ((n = sendto(gl.rd,b.b,len,0,&gl.idev.fnav.sa,sizeof(gl.idev.fnav.sa))) == len) return;
	/* there is a problem, maybe out of WiFi range ? */
	gl.idev.fnav.sin_port = 0;
	BLOG("NFWD,%d,%d,\"%s\"",len,n,n>0?"?":strerror(errno));
}

static void idv_in(void)
{
	int	nk, ns, na, psh;
	char	*s, *e, *a, *b, *av[8], dat[1028], msg[128];

	if ((nk = read(gl.cd,dat,sizeof(dat)-4)) <= 1) return;
	if (gl.idev.trcv == 0) SLOG("IDEV,1");
	gl.idev.trcv = gl.tnow;
	for (e = (b = dat) + nk; b < e; b++) {
		while (b < e && *b <= ' ') b++;
		for (s = b; b < e && *b >= ' '; b++); *b = 0;
		if ((b - s) < 4) continue;
		if (strbeg("AT*C",s) == 0) continue;
		if ((na = cmd2av(s+3,av,NEL(av))) != 4) {
			if (na > 0 && strequ(av[0],"COMWDG"))
				gl.idev.fcst &= ~AS_COM_WATCHDOG;
			continue;
		}
		if (strequ(av[0],"CTRL")) {
			/* CTRL <seq> <typ> 0
			 * 0    1     2     3  4 */
			if (av[2][0] == '5') { /* ack AS_COMMAND_ACK */
				gl.idev.fcst &= ~AS_COMMAND_ACK;
				continue;
			}
			sprintf(msg,"IDEV,CTRL,%s",av[2]);
			switch (av[2][0]) {
			case '4': a = gl.smgr.c4b, nk = gl.smgr.c4n; break;
			case '6': a = gl.smgr.c6b, nk = gl.smgr.c6n; break;
			default:  BLOG("%s,TYPE?",msg); continue; break;
			}
			if (gl.fd <= 0 || nk <= 0) 
				BLOG("%s,%s,%d",msg,gl.fd<0?"LINK?":"DATA?",nk);
			else if (gl.idev.fcst & AS_COMMAND_ACK)
				BLOG("%s,BUSY!",msg);
			else {
				ns = write(gl.fd,a,nk);
				BLOG("D_FD,CTRL,%s,send,%d,%d",av[2],nk,ns);
			}
			continue;
		}
		if (strequ(av[0],"CONFIG")) {
			/* CONFIG <seq> <var> <val>
			 * 0      1     2     3     4 */
			if (gl.idev.fcst & AS_COMMAND_ACK) {
				BLOG("IDEV,CONFIG,BUSY");
				continue; /* is busy as far as iDev is concerned */
			}
			gl.idev.fcst |= AS_COMMAND_ACK;
			smgr_c4bu(&gl.smgr,av[2],av[3]);
			if ((a = strbeg("custom:",av[2]))) {
				smgr_c4bf(&gl.smgr,a,av[3]);
				continue;
			}
			psh = 0;
			if ((a = strbeg("video:",av[2]))) {
				if (strbeg("video_chan",a)) {
					gl.vzap++; gl.vzap &= 03;
					sprintf(av[3]=msg,"%d",gl.vzap);
					psh++;
				}
				else if (!gl.rvid) psh++;
			}
			else if (strbeg("userbox:",av[2])) {
				if (!gl.rvid) psh++;
			}
			else if (strbeg("gps:",av[2])) psh++;
			else if ((a = strbeg("general:",av[2]))) {
				if (strbeg("navdata_",a)) {
					if (a[8] == 'd') gl.idev.nall = 1;
					else psh++;
				}
				else if (strbeg("local",a)) psh++;
			}
			if (psh) psh_cfg(av[2],av[3]);
			continue;
		}
	}
}

/*
 * iDev tcp data connection (5559)
 *
 * gl.id (permanent) iDev data port connect is redirected here through iptables nat
 * gl.fd (transient) iDev data port link (where to send data to)
 */
static void idd_in(void)
{
	int	fd, res;
	soa_t	fa;
	socklen_t fl;

	fl = sizeof(fa.sa);
	if ((fd = accept(gl.id,&fa.sa,&fl)) <= 0) return;
	while ((res = 1)) {
		if (ioctl(fd,FIONBIO,&res)) break;
		gl.fd = add_fd(fd);
		BLOG("D_FD,conn,%d",gl.fd);
		return;
	}
	BLOG("D_ID,error,%d,\"%s\"",res,strerror(errno));
	close(fd);
}

static void fdd_in(void)
{
	int	n;
	u08_t	dat[1024];

	n = read(gl.fd,dat,sizeof(dat));
	BLOG("D_FD,disc,%d,%d,\"%s\"",gl.fd,n,n<0?strerror(errno):"");
	if (n <= 0) gl.fd = del_soc(gl.fd);
}

static void ddd_in(void)
{
	int	n;
	char	buf[2048];

	if ((n = read(gl.dd,buf,sizeof(buf))) <= 0) {
		BLOG("D_DD,read,%d,\"%s\",%d",n,strerror(errno),gl.smgr.cex);
		gl.smgr.cex = 0;
		gl.dd = del_soc(gl.dd);
		return;
	}
	BLOG("D_DD,read,%d",n);
	switch (gl.smgr.cex) {
	case 4:
		if ((gl.smgr.c4n + n) >= NEL(gl.smgr.c4b)) {
			BLOG("D_DD,CVB,overflow");
			gl.smgr.cex = 0;
			gl.smgr.c4n = 0;
			break;
		}
		bcopy(buf,gl.smgr.c4b+gl.smgr.c4n,n); gl.smgr.c4n += n;
		if (buf[n-1]) break; /* there is more ... */
		BLOG("D_DD,EOF,%d,%d",gl.smgr.cex,gl.smgr.c4n);
		gl.smgr.cex = 0;
		smgr_c4bi(&gl.smgr);
		break;
	case 6:
		if ((gl.smgr.c6n + n) >= NEL(gl.smgr.c6b)) {
			BLOG("D_DD,CCB,overflow");
			gl.smgr.cex = 0;
			gl.smgr.c6n = 0;
			break;
		}
		bcopy(buf,gl.smgr.c6b+gl.smgr.c6n,n); gl.smgr.c6n += n;
		if (buf[n-1]) break; /* there is more ... */
		BLOG("D_DD,EOF,%d,%d",gl.smgr.cex,gl.smgr.c6n);
		gl.smgr.cex = 0;
		break;
	}
}

/*
 * rx2at input
 */
static void tx_disc(void)
{
	gl.ardu.disc = gl.tnow;
	gl.ardu.dcmd = gl.ardu.ucmd;
	gl.ardu.dseq = 0;
	SLOG("DISC,ALRM,%c",gl.ardu.dcmd.mode);
}

static void rx_cmd(int na, char **av)
{
	char	c, *s, *d = 0, cmd[256];
	vif_t	v[4];
	int	i, k;

	if (na < 2) return;
	if (av[0][1] != 'X') return;
	if (av[0][2]) return;

	switch ((c = av[1][0])) {
	case 'V':
		if (na < 3 || av[2][1]) break;
		switch ((c = av[2][0])) {
		case 'Z': /* video ZAP */
			gl.vzap++; gl.vzap &= 03;
			sprintf(cmd,"%d",gl.vzap);
			psh_cfg("video:video_channel",cmd);
			break;
		case '0': /* stop  video recording */
		case '1': /* start video recording */
			if (!gl.ard2) break;
			if (c == '1') {
				if (!gl.rvid || !gl.nt25.rvok) break;
				psh_cfg("video:video_codec","130");
				psh_cfg("userbox:userbox_cmd","1");
			}
			else {
				psh_cfg("video:video_codec","129");
				psh_cfg("userbox:userbox_cmd","0");
			}
			break;
#if	0
		case '2': /* snapshot */
			psh_cfg("userbox:userbox_cmd","2,0,0,2014_000");
			break;
#endif
		}
		break;
	case 'B': /* battery low signal from Arduino VLBA mechanism (1 - on, 0 - off) */
		if (na < 3) break;
		led(BLINK_RED,5.0,av[2][0]=='1'?0:1);
		break;
	case 'C': /* summary config */
		if (gl.ardu.acfg || na < 2) break;
		for (i = 2; i < na; i++) {
			s = 0;
			switch (i) {
			case 2: s = "control:outdoor"; break;
			case 3: s = "control:flight_without_shell"; break;
			case 4: s = "control:euler_angle_max"; break;
			case 5: s = "control:control_vz_max"; break;
			case 6: s = "control:control_yaw"; break;
			case 7: s = "control:altitude_max";
				k = strtol(av[i],0,0);
				if (k <=    0) k = gl.ard2 ? 1000 : 10;
				if (k <= 1000) k *= 1000;
				sprintf(av[i]=cmd,"%d",k);
				break;
			case 8: gl.anim = strtol(av[i],0,0);
				break;
			case 9: if (!gl.ard2) break;
				s = "video:video_on_usb";
				gl.rvid = strtol(av[i],0,0);
				break;
			case 10:
				gl.dcat = strtol(av[i],0,0);
				break;
			}
			if (s && av[i] && *av[i]) psh_cfg(s,av[i]);
		}
		gl.ardu.acfg = 1;
		break;
	case 'T': /* flat trimm */
		d = cmd + sprintf(cmd,"FTRIM=");
		led(BLINK_GREEN,5.0,1);
		SLOG("TRIM");
		break;
	case 'A': /* animation trigger */
		if (na < 3 || !gl.anim) break;
		if ((i = av[2][0]-'1') >= 0 && i < 4) {
			/* flip/nod animations
			 *     drone 2         drone 1
			 * S 0 17,15 bck flip  3,1000 bck nod
			 * W 1 18,15 lft flip  0,1000 lft nod
			 * N 2 16,15 fwd flip  2,1000 fwd nod
			 * E 3 19,15 rgt flip  1,1000 rgt nod
			 */
			if (gl.ard2) i += 4;
			sprintf(cmd,"%d,%d","\x03\x00\x02\x01\x11\x12\x10\x13"[i],gl.ard2?15:1000);
			psh_cfg("control:flight_anim",cmd);
		}
		break;
	case 'D': /* TX disconnect */
		if (gl.ardu.disc == 0) tx_disc();
		break;
	case 'E': /* emergency */
		if (gl.estp == 0) gl.estp = 1;	/* set */
		if (gl.estp == 3) gl.estp = 4;	/* clear */
		break;
	case '0': /* LAND */
	case '1': /* FM_1 */
	case '2': /* FM_2 */
	case '3': /* FM_3 */
		for (i = 0, k = 2; i < NEL(v) && k < na; i++, k++) {
			v[i].i = strtol(av[k],&s,10);
			if (*s) break;
		}
		if (i < 4) break;
		c = gl.ardu.ucmd.mode; gl.ardu.ucmd.mode = av[1][0];
		for (k = 0; k < 4; k++)
			if (v[k].i)
				gl.ardu.ucmd.rpgy[k].f = (float)v[k].i/1000.0;
			else	gl.ardu.ucmd.rpgy[k].i = 0;
		if (gl.ardu.disc) gl.ardu.reco = gl.tnow;
		else if (gl.estp) {
			if (gl.ardu.ucmd.mode == '0') {
				if (gl.estp == 2) gl.estp = 3;
				if (gl.estp == 5) gl.estp = 0;
			}
		}
		else if (gl.ardu.ucmd.mode != c) SLOG("FMOD,%c",gl.ardu.ucmd.mode);
		break;
	}
	if (d) at_send(cmd);
}

static void ard_in(void)
{
	int	n;
	char	*s, *e, *b, *av[32], dat[512];

	if ((n = read(gl.ad,dat,sizeof(dat))) <= 1) return;
	if (--n == 0) return; /* strip terminating '\n' */
	if (gl.ardu.trcv == 0) gl.ardu.link = 1, SLOG("LINK,1");
	else if (gl.ardu.link < 0) return;	/* most likely garbage */
	gl.ardu.trcv = gl.tnow;
	for (e = (s = dat) + n; s < e; s++) {
		while (s < e && *s <= ' ') s++;
		for (b = s; s < e && *s >= ' '; s++);
		if ((s - b) < 4) continue;
		*s = 0;
		     if (b[0] == 'R') rx_cmd(cmd2av(b,av,NEL(av)),av);
		else if (b[0] == '$') gps_in(b);
		else if (b[0] == 'B') bmp_in(b);
		else if (b[0] == 'M') m56_in(b);
		else if (b[0] == 'H') hmc_in(b);
	}
}

static void grab_ctrl(void)
{
	led(BLINK_ORANGE,5.0,1);	/* show user we are here */
	at_send("PMODE=,2\rMISC=,2,20,2000,3000"); /* FreeFlight does these... */
	psh_cfg("custom:session_id","-all");
	psh_cfg("CTRL","6");
	psh_cfg("CTRL","4");
	psh_cfg("CTRL","-");
	psh_cfg("custom:session_id",gl.smgr.sid);
	psh_cfg("custom:application_id",gl.smgr.aid);
	psh_cfg("custom:profile_id",gl.smgr.pid);
	if (gl.ard2) {
		psh_cfg("video:codec_fps","25");
		psh_cfg("video:video_codec","129");
		psh_cfg("video:max_bitrate","1500");
		psh_cfg("video:bitrate_ctrl_mode","1");
	}
	else	psh_cfg("video:video_codec","64");
	/* default configuration */
	psh_cfg("detect:detect_type","3");	/* no detections */
	psh_cfg("control:control_level","0");
	psh_cfg("general:navdata_demo","1");
	if (gl.ard2)
		/* 3 3 2 2  2 2 2 2  2 2 2 2  1 1 1 1  1 1 1 1  1 1                       */
		/* 1 0 9 8  7 6 5 4  3 2 1 0  9 8 7 6  5 4 3 2  1 0 9 8  7 6 5 4  3 2 1 0 */
		/* _ _ _ _  _ _ 1 _  _ _ _ _  _ _ _ _  _ _ _ _  _ _ _ _  _ _ _ _  _ _ _ 1 */
		psh_cfg("general:navdata_options","0x02000001");
}

typedef struct {	/* message for Arduino */
	u32_t	dcst;	/* drone status */
	char	cbat;	/* battery percentage left */
	char	beep;	/* tell Arduino to beep beep times */
	char	sgps;	/* GPS fix status */
}	__attribute__ ((packed)) m2a_t;

static void ard_sq(void)
{
	struct ardu	*a = &gl.ardu;
	cvv_t		*cvv;
	char		*d, cmd[128];
	m2a_t		m2a;

	if (a->disc) {
		switch (a->dseq) {
		case 0: /* just got activated, ucmd was copied to dcmd */
			a->reco = 0;
			bzero(a->dcmd.rpgy,sizeof(a->dcmd.rpgy));
			if (a->dcmd.mode >= '1' && a->dcmd.mode <= '3') {
				a->dcmd.mode = '1';
				if (a->link < 0) a->dtot = S2TIC(1);
				else if (gl.dcat <= 0) a->dtot = S2TIC(10);
				else a->dtot = gl.dcat * NTPS;
				a->dtot += gl.tnow;
				a->dseq = 2;
			}
			else a->dcmd.mode = '0', a->dseq = 1;
			break;
		case 1:	/* wait for reconnect on ground */
			if (a->reco && a->ucmd.mode == '0') a->disc = 0;
			break;
		case 2: /* wait for reconnect when airborne */
			if (a->reco) a->disc = 0;
			else if (gl.tnow > a->dtot) {
				SLOG("DISC,LAND");
				a->dcmd.mode = '0';
				a->dseq = 1;
			}
			break;
		}
		if (a->disc == 0) SLOG("DISC,RECO,%c",a->ucmd.mode);
	}

	/* watchdog */
	switch (a->swdg) {
	case 3: a->swdg = 0;
		/* FALLTROUGH */
	case 0: if (gl.navh.cst & AS_COM_WATCHDOG) {
			BLOG("CWDG,RESET");
			at_send("COMWDG=");
			gl.seq = 0;
			a->tout = gl.tnow + S2TIC(0.3);
			a->scfg = 0;
			a->swdg = 1;
		}
		else if (a->link < 0) break;
		else if (a->trcv == 0) {
			write(gl.ad,"\002",1);
			SLOG("C_SC");
			a->tout = gl.tnow + S2TIC(0.1);
			a->swdg = 2;
		}
		else if ((gl.tnow - a->trcv) > S2TIC(1.0)) {
			a->link = -1; SLOG("LINK,-1");	/* fell off ? */
			if (a->disc == 0) tx_disc();
		}
		break;
	case 1: if (!(gl.navh.cst & AS_COM_WATCHDOG) || gl.tnow >= a->tout) a->swdg = a->scfg = 0;
		break;
	case 2: if (a->trcv != 0 || gl.tnow > a->tout) a->swdg = 3;
		break;
	}

	/* config fifo */
	switch (a->scfg) {
	case 0: if (a->swdg) break;
		a->scfg = gl.navh.cst & AS_COMMAND_ACK ? 3 : 1;
		break;
	case 1: if ((cvv = nxt_cfg()) == 0) {
			if (a->acfg) {
				a->acfg = 0;
				led(BLINK_GREEN,5.0,1);
				write(gl.ad,"\001",1);	/* send config ACK to Arduino */
			}
			break;
		}
		if (gl.nnav == 0) break;
		if (strequ(cvv->var,"CTRL")) {
			if (gl.dd <= 0) {
				if ((gl.dd = add_soc(SOCK_STREAM,&gl.da,1)) > 0)
					connect(gl.dd,&gl.ddat.sa,sizeof(gl.ddat.sa));
				else BLOG("D_DD,sock,%s",strerror(errno));
				a->scfg = 4;
				break;
			}
			if (gl.smgr.cex != 0) { pop_cfg(); break; }
			switch (cvv->val[0]) {
			case '4': gl.smgr.cex = 4; gl.smgr.c4n = 0; break;
			case '6': gl.smgr.cex = 6; gl.smgr.c6n = 0; break;
			case '-': gl.dd = del_soc(gl.dd); break;
			}
			if (gl.smgr.cex == 0) { pop_cfg(); break; }
			sprintf(cmd,"CTRL=,%d,0",gl.smgr.cex);
			BLOG("%s",cmd);
			at_send(cmd);
			a->tout = gl.tnow + S2TIC(3.0);
		}
		else {
			sprintf(cmd,"CONFIG=,\"%s\",\"%s\"",cvv->var,cvv->val);
			SLOG("DCFG%s",cmd+7);
			at_send(cmd);
			a->tout = gl.tnow + S2TIC(0.5);
		}
		a->scfg = 2;
		break;
	case 2: if (gl.navh.cst & AS_COMMAND_ACK) a->scfg = gl.smgr.cex ? 3 : 5;
		else if ((int)(gl.tnow - a->tout) >= 0) a->scfg = 4;
		break;
	case 3: if (gl.smgr.cex == 0) a->scfg = 5;
		else if ((int)(gl.tnow - a->tout) >= 0) a->scfg = 4;
		break;
	case 4: /* deal with retries */
		gl.smgr.cex = 0;
		if ((a->rcfg += 1) >= 3) a->rcfg = pop_cfg();
		a->scfg = 6;
		break;
	case 5: a->rcfg = pop_cfg();
		/* FALLTHROUGH */
	case 6: at_send("CTRL=,5,0");
		a->tout = gl.tnow + S2TIC(0.50);
		a->scfg = 7;
		break;
	case 7: if (!(gl.navh.cst & AS_COMMAND_ACK)) a->scfg = 1;
		else if ((int)(gl.tnow - a->tout) >= 0) a->scfg = 6;
		break;
	}

	/* data for Arduino */
	if (gl.tnow >= a->sdto && a->link > 0) {
		a->sdto = gl.tnow + S2TIC(0.50);
		m2a.dcst = gl.navh.cst;
		m2a.cbat = gl.nt00.bat;
		m2a.beep = 0;
		m2a.sgps = 0;
		d = cmd;
		*d++ = '$';
		d = hexstr(&m2a,sizeof(m2a),d);
		*d++ = '*';
		*d++ = '\n';
		write(gl.ad,cmd,d-cmd);
	}
}

static void usr_cmd(void)
{
	pcmd_t	*p;
	char	*d, c, cmd[128];
#define	REFLAND	0x11540000
#define	REFSTRT	0x11540200
#define	REFESTP	0x11540100

	d = cmd;
	if (gl.estp) {
		c = 0;
		switch (gl.estp) {
		case 1: if (gl.navh.cst & AS_EMERGENCY) gl.estp = 2; else c = 1; break;
		case 4: if (gl.navh.cst & AS_EMERGENCY) c = 1; else gl.estp = 5; break;
		}
		sprintf(d,"REF=,%u",c?REFESTP:REFLAND);
	}
	else {
		p = gl.ardu.disc ? &gl.ardu.dcmd : &gl.ardu.ucmd;
		if ((c = p->mode) >= '1' && c <= '3') {
			d += sprintf(d,"PCMD%s=,1",gl.ard2?"_MAG":"");
			if (c == '1' && p->rpgy[0].i == 0 && p->rpgy[1].i == 0) d[-1] = '0';
			d += sprintf(d,",%d,%d,%d,%d%s",
				p->rpgy[0].i,p->rpgy[1].i,p->rpgy[2].i,p->rpgy[3].i,
				gl.ard2?",0,0":"");
			BLOG(cmd);
			*d++ = '\r';
		}
		else c = 0;
		sprintf(d,"REF=,%u",c?REFSTRT:REFLAND);
	}
#undef	REFLAND
#undef	REFSTRT
#undef	REFESTP
	at_send(cmd);
}

static void loop(void)
{
	fd_set	rd;
	tmv_t	tv;
	int	n, u, s;

	gl.tnow = gl.tnav = tnow();
	grab_ctrl();
	n = 1, sendto(gl.rd,&n,sizeof(n),0,&gl.dnav.sa,sizeof(gl.dnav.sa));
	tv.tv_sec = 0;
	SLOG("LOOP,1");
	u = 0;
	while (!gl.exit) {
		if (gl.idev.trcv && (gl.tnow - gl.idev.trcv) >= S2TIC(30.0)) {
			SLOG("IDEV,0");
			gl.idev.fcst |= AS_COM_WATCHDOG;
			gl.idev.fnav.sin_port = 0;
			gl.idev.nall = 0;
			gl.idev.trcv = 0;
		}
		ard_sq();
		if (u) u--, usr_cmd();
		else if (gl.tnav && (gl.tnow - gl.tnav) > S2TIC(1.0))
			gl.tnav = gl.nnav = 0, SLOG("LOOP,-1!");
		if (gl.nat > 0 && gl.nnav) {
			n = sendto(gl.cd,gl.atb,gl.nat,0,&gl.dcmd.sa,sizeof(gl.dcmd.sa));
			if (n != gl.nat) SLOG("LOOP,\"%s\"",strerror(errno));
		}
		gl.nat = 0;
		rd = gl.sd;
		tv.tv_usec = 100000;
		n = select(gl.md,&rd,0,0,&tv);
		if (n < 0 && errno != EINTR) break;
		gl.tnow = tnow();
		if (n > 0 && (s = gl.ad) > 0 && FD_ISSET(s,&rd)) n--, ard_in();
		if (n > 0 && (s = gl.cd) > 0 && FD_ISSET(s,&rd)) n--, idv_in();
		if (n > 0 && (s = gl.rd) > 0 && FD_ISSET(s,&rd)) n--, nav_in(), u++;
		if (n <= 0) continue;
		if (n > 0 && (s = gl.nd) > 0 && FD_ISSET(s,&rd)) n--, nrq_in();
		if (n > 0 && (s = gl.dd) > 0 && FD_ISSET(s,&rd)) n--, ddd_in();
		if (n > 0 && (s = gl.id) > 0 && FD_ISSET(s,&rd)) n--, idd_in();
		if (n > 0 && (s = gl.fd) > 0 && FD_ISSET(s,&rd)) n--, fdd_in();
	}
	SLOG("LOOP,0");
}

static void ipt_clr(void)
{
	system("iptables -P INPUT ACCEPT ; iptables -F ; iptables -t nat -F");
}

static int ipt_natrri(int prot, int port, int palt)
{
	char	buf[256];

	sprintf(buf,"iptables -t nat -A PREROUTING -p %s ! -s 127.0.0.1 --dport %d -j DNAT --to :%d",
		prot==IPPROTO_TCP?"tcp":"udp",ntohs(port),ntohs(palt));
	return system(buf);
}

static void ipt_blks(void)
{
	int	fd, n;
	char	*p, *b, *s, *e, *r, buf[1024], cmd[256];

	p = strtxf(
	"netstat -ln | egrep \"^udp|^tcp\" | sed \"s= [ ]*= =g\" "
	"| cut -d ' ' -f 1,4 | sed \"s=0.0.0.0:==\" > ",buf,sizeof(buf));
	sprintf(p,"/tmp/tmp%06d",getpid());
	system(buf);
	fd = open(p,0); unlink(p);
	if (fd < 0) return;
	n = read(fd,buf,sizeof(buf)-2); close(fd);
	for (e = (s = buf) + n; s < e; s = b) {
		while (s < e && *s <= ' ') s++;
		for (b = s; b < e && *b > ' '; b++); *b++ = 0;
		for (p = b; b < e && *b > ' '; b++); *b++ = 0;
		if ((n = strtol(p,&r,10)) <= 0 || *r) continue; /* FIXME ssh 'tcp6 :::22' */
		if (n == 21 || n == 23 || n == 67 || n == 514) continue; /* standard ftp, telnet, dhcpd, rshd */
		if (*s == 'u') {
//			if (n ==  5552) continue; /* authentication port ? */
			if (n ==  5554) continue; /* navdata port, gets rerouted */
			if (n ==  5555) continue; /* video port (D1) */
			if (n ==  5556) continue; /* comand port, gets rerouted */
//			if (n == 14551) continue; /* mavlink */
		}
		else if (*s == 't') {
			if (n ==  5551) continue; /* local ftp server root: /update */
			if (n ==  5553) continue; /* video recording (D2) */
			if (n ==  5555) continue; /* video streaming (D2) */
//			if (n ==  5557) continue; /* ? */
			if (n ==  5559) continue; /* data, gets rerouted for editing */
		}
		sprintf(cmd,"iptables -A INPUT -i ath0 -p %s --dport %s -j DROP",s,p);
		BLOG("BLCK,%d,%s:%s",system(cmd),s,p);
	}
}

static void sleep_10ms(int n)
{
	tmv_t	tv;

	/* the dispatch is what is important here */
	tv.tv_sec = 0;
	do tv.tv_usec = 10000, select(0,0,0,0,&tv); while (--n > 0);
}

static int udp5556_traffic(void)
{
	int	fd, n = 0;
	char	*p, lpf[16], buf[1024];

	sprintf(lpf,"CK5556_%d",getpid());
	sprintf(buf,"iptables -A INPUT -p udp --dport 5556 -j LOG --log-prefix \"%s \"",lpf);
	system(buf);
	sleep_10ms(10);
	buf[10] = 'D';
	system(buf);

	p = buf + sprintf(buf,"dmesg | grep \"%s \" | tail -1 > ",lpf);
	sprintf(p,"/tmp/%s.udp",lpf);
	system(buf);
	fd = open(p,0); unlink(p);
	if (fd >= 0) n = read(fd,buf,sizeof(buf)-2), close(fd);
	return (fd > 0 && n == 0) ? 0 : -1;
}

void cleanup(void)
{
	int	s;

	SLOG("EXIT,\"%s\"",version+5);
	ipt_clr();
	if ((s = gl.cd) > 0) gl.cd = del_soc(s);
	if ((s = gl.nd) > 0) gl.nd = del_soc(s);
	if ((s = gl.rd) > 0) gl.rd = del_soc(s);
	if ((s = gl.dd) > 0) gl.dd = del_soc(s);
	if ((s = gl.id) > 0) gl.id = del_soc(s);
	if ((s = gl.fd) > 0) gl.fd = del_soc(s);
	if ((s = gl.ad) > 0) {
		gl.ad = -1;
		write(s,"\004",1);
		tcsetattr(s,TCSADRAIN,gl.ap);
		close(s);
	}
	if ((s = gl.ld) > 0) gl.ld = -1, close(s);
}

int main(int ac, char **av, char **env)
{
	char	*p;
	int	s;

	bzero(&gl,sizeof(gl));
	FD_ZERO(&gl.sd);
	gl.smgr.sid = "00000001"; /* some number != 0 will do */
	gl.smgr.pid = "6fb0c592"; /* crc32 of 'mirumod' */
	gl.smgr.aid = "078222bc"; /* crc32 of 'mirumod:0.22' */
	if (tref()) return errno;
	for (s = 0; ++s < ac; )
		if (*(p = av[s]) == '-')
			while (*(++p)) switch (*p) {
			case 'w': gl.flag |= F_BLK_WIFI; break;
			case 'l': if (++s < ac) gl.lgi = strtol(av[s],0,0); break;
			}
#if defined(FLASH)
	gl.ld = open("/tmp/at2so.log",O_CREAT|O_TRUNC|O_WRONLY,0664);
#else
	sprintf(gl.lgb,"/data/video/P%d.txt",gl.lgi%5);
	gl.ld = open(gl.lgb,O_CREAT|O_TRUNC|O_WRONLY,0664);
#endif
	SLOG("BOOT,\"%s\",%d",version+5,gl.lgi);

	if ((s = open("/update/version.txt",0)) > 0) {
		read(s,gl.dfmw,sizeof(gl.dfmw)-1);
		close(s);
		for (p = gl.dfmw, s = NEL(gl.dfmw); --s >= 0 && *p; p++)
			if (*p == '\r' || *p == '\n') *p = 0;
		SLOG("DFMW,%s",gl.dfmw);
		if ((s = strtol(gl.dfmw,&p,10)) >= 2) gl.ard2 = 1;
		else if (s != 1 || *p != '.' || strtol(p+1,0,10) < 7) return -1;
	}

	gl.dcmd.sin_fmly = AF_INET;
	gl.dcmd.sin_addr = IPV4A(127,0,0,1); /* localhost */
	gl.dnav = gl.ddat = gl.dcmd;
	gl.dcmd.sin_port = htons(5556);
	gl.dnav.sin_port = htons(5554);
	gl.ddat.sin_port = htons(5559);

	/* serial receiver (Arduino)
	 * using ICANON without ICRNL, -> read/select complete when '\n' is received */
	if ((gl.ad = open("/dev/tty",2)) < 0) return errno;
	if (tcgetattr(gl.ad,gl.ap)) return errno;
	gl.ap[1] = gl.ap[0];
	gl.ap[1].c_iflag = IGNBRK|IGNPAR|ISTRIP;
	gl.ap[1].c_lflag = ICANON; 	
	if (tcsetattr(gl.ad,TCSANOW,&gl.ap[1])) return errno;
	add_fd(gl.ad);

	atexit(cleanup);

	if (udp5556_traffic()) return -1;

	ipt_clr();
	ipt_blks();	/* block anything incoming to unknown server ports... */

	/* sockets */
	if ((gl.rd = add_soc(SOCK_DGRAM ,&gl.ra,1)) < 0) return errno; /* int: sink for navdata from drone */
	if ((gl.cd = add_soc(SOCK_DGRAM ,&gl.ca,1)) < 0) return errno; /* ext/int: sink/src for drone commands */
	if ((gl.nd = add_soc(SOCK_DGRAM ,&gl.na,1)) < 0) return errno; /* ext: navdata  */
	if ((gl.id = add_soc(SOCK_STREAM,&gl.ia,1)) < 0) return errno; /* ext: data/ctrl port */
	listen(gl.id,1);

	if (gl.flag & F_BLK_WIFI) {
		if (gl.ard2)
			system("env WORKAREA=/lib/firmware ATH_PLATFORM=parrot-omap-sdio loadAR6000.sh unloadall");
		else	system("iptables -A INPUT -i ath0 -j DROP");
	}
	else {
		/* reroute critical packets incoming over Wi-Fi */
		if (ipt_natrri(IPPROTO_UDP,gl.dcmd.sin_port,gl.ca.sin_port)) return errno;
		if (ipt_natrri(IPPROTO_UDP,gl.dnav.sin_port,gl.na.sin_port)) return errno;
		if (ipt_natrri(IPPROTO_TCP,gl.ddat.sin_port,gl.ia.sin_port)) return errno;
	}

	loop();

	return errno;
}

