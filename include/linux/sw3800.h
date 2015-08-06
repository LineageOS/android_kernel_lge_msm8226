#ifndef __SW3800_H__
#define __SW3800_H__

#define AUTH_IC_DEV_NAME "sw3800"
#define COVER_SW3800_TYPE_ERR 0
#define COVER_SW3800_SHA_ERR 0

#define sectet_key1 0xE0873BC1
#define sectet_key2 0x70D9104B

struct sw3800_platform_data {
	int backcover_detect_pin;
	int backcover_validation_pin;
	int cover_pullup_pin;
	unsigned int backcover_irq;
	unsigned long irq_flags;
};

typedef struct _aka_cover_info aka_cover_info;
struct _aka_cover_info{
  u8 challenge_data[8];
};


#define BCF		0
#define nBCF	1
#define D9		2
#define D8		3
#define D7		4
#define D6		5
#define D5		6
#define D4		7
#define P3		8
#define D3		9
#define D2		10
#define D1		11
#define P2		12
#define D0		13
#define P1		14
#define P0		15
#define INV		16

#define SDA		0x01
#define WRA		0x02
#define WD		0x03
#define RBE		0x04
#define RBL		0x04
#define RRA		0x05

#define DATA_WORD			17
#define INVERSION_CHK		16
#define SECRETKEY_MSB		0x00000000
#define SECRETKEY_LSB		0x00000000
#define HALF_DATA_BIT		7
#define ONE_TAU				20
#define THREE_TAU			60
#define STOP_TAU			100
#define DATA_BIT				360
#define ERROR_CNT			10000
#define RESPONSE_AUTH		20
#define TRAINING_SEQ		4
#define TOTAL_WORD			18
#define ONE_TAU_MIN			4	//0.4
#define ONE_TAU_MAX		16	//1.6
#define THREE_TAU_MIN		24	//2.4
#define THREE_TAU_MAX		36	//3.6


#ifndef _SHA_enum_
#define _SHA_enum_
	enum
	{
		shaSuccess = 0,
		shaNull,			/* Null pointer parameter */
		shaInputTooLong,	/* input data too long */
		shaStateError		/* called Input after Result */
	};
#endif

#define SHA1HashSize 20

/*
 *	This structure will hold context information for the SHA-1
 *	hashing operation
 */
typedef struct SHA1Context
{
	u32 Intermediate_Hash[SHA1HashSize/4]; /* Message Digest  */

	u32 Length_Low;			/* Message length in bits	   */
	u32 Length_High;			/* Message length in bits	   */

							   /* Index into message block array   */
	int Message_Block_Index;
	u8 Message_Block[64];		/* 512-bit message blocks	   */

	int Computed;				/* Is the digest computed?		   */
	int Corrupted;			   /* Is the message digest corrupted? */
} SHA1Context;

int SW3800_Authentication(void);
void bifTrans_Command(u8 Transaction, u8 addr_data);
int SiliconWorks_Authentication(void);
void bifTransReadMultUint8(u8 Readtime);
int bifTransReadUint(void);

int SHA1Reset(SHA1Context *);
int SHA1Input(SHA1Context *, const u8 *, unsigned int);
int SHA1Result(SHA1Context *context, u8 Message_Digest[SHA1HashSize], u32 Secret_key0, u32 Secret_key1);
int SHA2Result(SHA1Context *, u8 Message_Digest[SHA1HashSize], u8 Secret_Digest[SHA1HashSize]);
void SHA1PadMessage(SHA1Context *context, u32 secret_key, u32 secret_key1);
void SHA2PadMessage(SHA1Context *context, u8 Secret_Digest[SHA1HashSize]);
void SHA1ProcessMessageBlock(SHA1Context *);
void Apply_SHA1(u32 Secret_key0, u32 Secret_key1, aka_cover_info *ci);

void cover_challenge_data_init(aka_cover_info *ci);

void set_cover_id_pin_input_mode(void);
u8 get_cover_id_value(void);
extern void pm8xxx_slide_enable(void);
extern void pm8xxx_slide_disable(void);
extern void pm8xxx_slide_boot_func(void);
extern bool slide_boot_mode(void);

//Output
void set_cover_id_pin_output_mode(void);
void set_cover_id_pin_high(void);
void set_cover_id_pin_low(void);
void set_cover_pullup_pin_output_mode(void);
void set_cover_pullup_pin_high(void);
void set_cover_pullup_pin_low(void);
//Random( for seed )
int get_random_4byte(void);



#endif /* __SW3800_H__ */
