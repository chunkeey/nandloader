
#include <linux/serial_reg.h>
#include <linux/smp.h>
#include <asm/addrspace.h>
#include <asm/byteorder.h>
#include <asm/page.h>
#include <asm/mipsregs.h>
#include <asm/r4kcache.h>
#include <asm/cacheops.h>

#include <ath79.h>
#include <ar71xx_regs.h>
#include "nandloader_common.h"
#include "malloc_compat.h"

#define ATH_DDR_CTL_CONFIG          ATH_DDR_CTL_BASE+0x108
#define ATH_DDR_DDR2_CONFIG         ATH_DDR_CTL_BASE+0xb8
#define ATH_DDR_BURST               ATH_DDR_CTL_BASE+0xc4
#define ATH_DDR_BURST2              ATH_DDR_CTL_BASE+0xc8
#define ATH_AHB_MASTER_TIMEOUT      ATH_DDR_CTL_BASE+0xcc

#define ATH_DDR_CTL_CONFIG_HALF_WIDTH (1 << 1)

#define ATH_DDR2_CONFIG_DDR2_EN        1

#define ATH_DDR_CONTROL_EMR3S         (1 << 5)
#define ATH_DDR_CONTROL_EMR2S         (1 << 4)
#define ATH_DDR_CONTROL_PREA          (1 << 3)
#define ATH_DDR_CONTROL_REF           (1 << 2)
#define ATH_DDR_CONTROL_EMRS          (1 << 1)
#define ATH_DDR_CONTROL_MRS           (1 << 0)

#define ATH_DDR_REFRESH_ENABLE        (1 << 14)
#define ATH_DDR_REFRESH_PERIOD        (0x3fff)

extern void __attribute__((weak)) qca955x_lowlevel_init(void);
extern void __attribute__((weak)) qca955x_init_ddr(void);
extern void __attribute__((weak)) qca955x_init_pcie_plls(void);
extern void __attribute__((weak)) ar934x_init_usb_phy(void);
extern void __attribute__((weak)) ar934x_lowlevel_init(void);
extern void __attribute__((weak)) ar934x_init_ddr(void);
extern void __attribute__((weak)) ar934x_init_pcie_plls(void);

struct _uart_ptr {
	volatile uint32_t rbr_thr;
	volatile uint32_t ier;
	volatile uint32_t iir_fcr;
	volatile uint32_t lcr;
	volatile uint32_t mcr;
	volatile uint32_t lsr;
	volatile uint32_t msr;
	volatile uint32_t scr;
	volatile uint32_t usr;
};

#define UART_BASE ((struct _uart_ptr *)(KSEG1ADDR(AR71XX_UART_BASE)))


u32  ref_clock_rate = 0;

static inline void
init_xtimer(void)
{
   u32 reg, mask, bootstrap;

   if (soc_is_qca955x()) {
      reg = QCA955X_RESET_REG_BOOTSTRAP; mask = QCA955X_BOOTSTRAP_REF_CLK_40;
   } else {
      reg = AR934X_RESET_REG_BOOTSTRAP; mask = AR934X_BOOTSTRAP_REF_CLK_40;
   }

   bootstrap = ath79_reset_rr(reg);
   if (bootstrap & mask)
      ref_clock_rate = (40 * 1000 * 1000);
   else
      ref_clock_rate = (25 * 1000 * 1000);
}

static inline u32
mips_count_get(void)
{
    u32 count;
    asm volatile ("mfc0 %0, $9" : "=r" (count) :);
    return count;
}

void
udelay(unsigned long usec)
{
    ulong tmo;
    ulong start = mips_count_get();

    tmo = usec * (ref_clock_rate / 1000000);
    while ((ulong)((mips_count_get() - start)) < tmo)
        /*NOP*/;
}


static inline void
init_uart(void)
{
    u32 div = (ref_clock_rate) / (16 * 115200);

    UART_BASE->lcr = UART_LCR_DLAB;
    UART_BASE->rbr_thr = div & 0xff;
    UART_BASE->ier = div >> 8;
    UART_BASE->lcr = UART_LCR_WLEN8;
}

static void
uart_putc(char c)
{
    if (c == '\n')
	uart_putc('\r');	/* CR before LF, I mean handle newline */

    while ((UART_BASE->lsr & UART_LSR_THRE) == 0) ;

    UART_BASE->rbr_thr = c;
}

void
uart_puts(const char *str)
{
    while (*str)
    uart_putc(*str++);
}

int
uart_tstc(void)
{
    return (UART_BASE->lsr & UART_LSR_DR);
}

char
uart_getc(void)
{
    while(!uart_tstc());

    return UART_BASE->rbr_thr;
}

/*
 * Just walk memory 4MB at a time until we run wrap around
 */
uint32_t
ddr_find_size(void)
{
#define DDR_SIZE_INCR (4*1024*1024)

    uint8_t *p = (uint8_t *)KSEG1, pat = 0x77;
    uint32_t i;

    *p = pat;

    for (i = 1; ; i++) {
        /*
         * When we finally reach the end of RAM we wrap back to the
         * beginnig.  Thus, (p + i * DDR_SIZE_INCR) and p are
         * referencing the same piece of memory.
         */
        *(p + i * DDR_SIZE_INCR) = (uint8_t)(i);
        if (*p != pat) {
            break;
        }
    }

    return (i*DDR_SIZE_INCR);
}

//given an int containing the (I|D)S (I|D)L (I|D)A bits in the least significant positions, return the size
static inline void
get_cache_size(u_int32_t *isize, u_int32_t *dsize)
{
    u_int32_t conf1 = read_c0_config1();

    int ia = (conf1 & MIPS_CONF1_IA) >> 16,
        il = (conf1 & MIPS_CONF1_IL) >> 19,
        is = (conf1 & MIPS_CONF1_IS) >> 22,
        da = (conf1 & MIPS_CONF1_DA) >> 7,
        dl = (conf1 & MIPS_CONF1_DL) >> 10,
        ds = (conf1 & MIPS_CONF1_DS) >> 13;

    (*isize) = (64 << is) * (2 << il) * (ia + 1);
    (*dsize) = (64 << ds) * (2 << dl) * (da + 1);
}

static inline void
init_dram_uncached(u_int32_t *uncached_ram, u_int32_t cache_size)
{
    register u_int32_t *addr;

    ANNOUNCE_PROGRESS();

    for (addr = uncached_ram; addr < uncached_ram + cache_size; addr += 16) {
        addr[0] = 0;
        addr[1] = 0;
        addr[2] = 0;
        addr[3] = 0;
        addr[4] = 0;
        addr[5] = 0;
        addr[6] = 0;
        addr[7] = 0;
        addr[8] = 0;
        addr[9] = 0;
        addr[10] = 0;
        addr[11] = 0;
        addr[12] = 0;
        addr[13] = 0;
        addr[14] = 0;
        addr[15] = 0;
    }

    ANNOUNCE_OK();
}


static inline void
init_icache(u_int32_t *cached_ram, u_int32_t cache_size, u_int32_t line_size)
{
    register u_int32_t *addr;

    ANNOUNCE_PROGRESS();

    write_c0_taglo(0);       /* Ensure valid bit clear and parity consistent */

    for (addr = cached_ram; addr < cached_ram + cache_size; addr += line_size) {
        cache_op(Index_Store_Tag_I, addr); /* Clear tag to invalidate */
        cache_op(Fill, addr);              /* Fill so parity is correct */
        cache_op(Index_Store_Tag_I, addr); /* Invalidate to be safe */
    }

    ANNOUNCE_OK();
}

static inline void
init_dcache(u_int32_t *cached_ram, u_int32_t cache_size, u_int32_t line_size)
{
    register u_int32_t *addr;
    register u_int32_t dummy;

    ANNOUNCE_PROGRESS();

    write_c0_taglo(0);       /* Ensure valid bit clear and parity consistent */

    for (addr = cached_ram; addr < cached_ram + cache_size; addr += line_size) {
        cache_op(Index_Store_Tag_D, addr); /* Clear tag to invalidate */
    }

    for (addr = cached_ram; addr < cached_ram + cache_size; addr += line_size) {
        dummy = *(volatile u_int32_t *)addr; /* Read from each cache line */
    }

    for (addr = cached_ram; addr < cached_ram + cache_size; addr += line_size) {
        cache_op(Index_Store_Tag_D, addr); /* Invalidate to be safe */
    }

    ANNOUNCE_OK();
}

static inline void
enable_caches(void)
{
    u_int32_t config;

    ANNOUNCE_PROGRESS();

    config = read_c0_config();
    config = (config & ~CONF_CM_CMASK) | CONF_CM_CACHABLE_NONCOHERENT;
    write_c0_config(config);

    ANNOUNCE_OK();
}

#define LOWMEM_PRESERVE_OFFSET 0x00060000

static inline void
init_caches(void)
{
    u_int32_t isize, dsize;

    get_cache_size(&isize, &dsize);
    printf("D-cache size: %dK\n", isize >> 10);
    printf("I-cache size: %dK\n", dsize >> 10);

    init_dram_uncached((u_int32_t *)KSEG1 + LOWMEM_PRESERVE_OFFSET,
		       (isize > dsize ? isize : dsize)/sizeof(u_int32_t));
    init_icache((u_int32_t *)KSEG1 + LOWMEM_PRESERVE_OFFSET, isize,
		32/(sizeof (u_int32_t)));
    init_dcache((u_int32_t *)KSEG1 + LOWMEM_PRESERVE_OFFSET, dsize,
		32/(sizeof (u_int32_t)));
    enable_caches();
}


extern int nand_flash_init(void);
extern int nand_read(unsigned int, unsigned int, uint8_t *);
extern int nand_load_fw(void);
extern void printf_init(void(*)(char c));

static int
test_memory(void* addr)
{
    volatile register u_int32_t *ram = (volatile uint32_t *)addr;
    register int count;

    ANNOUNCE_PROGRESS();
    ram[0] = 0;
    ram[1] = ~0;
    ram[2] = 0x5a5a5a5a;
    ram[3] = 0xa5a5a5a5;
    ram[4] = 0xfffefffe;

#define RAMCHECK(off, val) do { if (ram[off] != val) { printf("failed RAM BORKED: (%p) 0x%x != 0x%x\n", &ram[off], ram[off], val); return -1; } } while(0)

    for (count = 0; count < 5; count++) {
        RAMCHECK(0, 0);
        RAMCHECK(1, ~0);
        RAMCHECK(2, 0x5a5a5a5a);
        RAMCHECK(3, 0xa5a5a5a5);
        RAMCHECK(4, 0xfffefffe);
    }

    ANNOUNCE_OK();
    return 0;
}
unsigned int ath79_soc;
void __iomem *ath79_reset_base;
void __iomem *ath79_pll_base;
void __iomem *ath79_ddr_base;

static void init_sys_type(void)
{
   u32 id;
   u32 major;

   ath79_reset_base = ioremap_nocache(AR71XX_RESET_BASE,
                  AR71XX_RESET_SIZE);
   ath79_pll_base = ioremap_nocache(AR71XX_PLL_BASE,
                AR71XX_PLL_SIZE);
   ath79_ddr_base = ioremap_nocache(AR71XX_DDR_CTRL_BASE,
                AR71XX_DDR_CTRL_SIZE);

   id = ath79_reset_rr(AR71XX_RESET_REG_REV_ID);
   major = id & REV_ID_MAJOR_MASK;

   switch (major) {

      case REV_ID_MAJOR_AR9341:
         ath79_soc = ATH79_SOC_AR9341;
         break;

      case REV_ID_MAJOR_AR9342:
         ath79_soc = ATH79_SOC_AR9342;
         break;

      case REV_ID_MAJOR_AR9344:
         ath79_soc = ATH79_SOC_AR9344;
         break;

      case REV_ID_MAJOR_QCA9556:
         ath79_soc = ATH79_SOC_QCA9556;
         break;

      case REV_ID_MAJOR_QCA9558:
         ath79_soc = ATH79_SOC_QCA9558;
         break;

      default:
          BUG();
   }
}

#ifdef MR18_SGMII_CAL
/* used for eth calibration */
#define MR18_OTP_BASE			(AR71XX_APB_BASE + 0x130000)
#define MR18_OTP_SIZE			(0x2000) /* just a guess */
#define MR18_OTP_MEM_0_REG		(0x0000)
#define MR18_OTP_INTF2_REG		(0x1008)
#define MR18_OTP_STATUS0_REG		(0x1018)
#define MR18_OTP_STATUS0_EFUSE_VALID	BIT(2)

#define MR18_OTP_STATUS1_REG		(0x101c)
#define MR18_OTP_LDO_CTRL_REG		(0x1024)
#define MR18_OTP_LDO_STATUS_REG		(0x102c)
#define MR18_OTP_LDO_STATUS_POWER_ON	BIT(0)

static int mr18_extract_sgmii_res_cal(void)
{
	void __iomem *base;
	unsigned int reversed_sgmii_value;

	unsigned int otp_value, otp_per_val, rbias_per, read_data;
	unsigned int rbias_pos_or_neg;
	unsigned int sgmii_res_cal_value;
	int res_cal_val;

	base = ioremap_nocache(MR18_OTP_BASE, MR18_OTP_SIZE);

	__raw_writel(0x7d, base + MR18_OTP_INTF2_REG);
	__raw_writel(0x00, base + MR18_OTP_LDO_CTRL_REG);

	while (__raw_readl(base + MR18_OTP_LDO_STATUS_REG) &
		MR18_OTP_LDO_STATUS_POWER_ON);

	__raw_readl(base + MR18_OTP_MEM_0_REG + 4);

	while (!(__raw_readl(base + MR18_OTP_STATUS0_REG) &
		MR18_OTP_STATUS0_EFUSE_VALID));

	read_data = __raw_readl(base + MR18_OTP_STATUS1_REG);

	if (!(read_data & 0x1fff))
		printf("OTP data is likely bad... but continuing\n");

	if (read_data & 0x00001000)
		otp_value = (read_data & 0xfc0) >> 6;
	else
		otp_value = read_data & 0x3f;

	if (otp_value > 31) {
		otp_per_val = 63 - otp_value;
		rbias_pos_or_neg = 1;
	} else {
		otp_per_val = otp_value;
		rbias_pos_or_neg = 0;
	}

	rbias_per = otp_per_val * 15;

	if (rbias_pos_or_neg == 1)
		res_cal_val = (rbias_per + 34) / 21;
	else if (rbias_per > 34)
		res_cal_val = -((rbias_per - 34) / 21);
	else
		res_cal_val = (34 - rbias_per) / 21;

	sgmii_res_cal_value = (8 + res_cal_val) & 0xf;

	reversed_sgmii_value  = (sgmii_res_cal_value & 8) >> 3;
	reversed_sgmii_value |= (sgmii_res_cal_value & 4) >> 1;
	reversed_sgmii_value |= (sgmii_res_cal_value & 2) << 1;
	reversed_sgmii_value |= (sgmii_res_cal_value & 1) << 3;
	printf("SGMII cal value = 0x%x\n", reversed_sgmii_value);
	return reversed_sgmii_value;
}

#define QCA955X_RESET_SGMII_ANALOG			BIT(12)
#define QCA955X_RESET_SGMII				BIT(8)
#define QCA955X_GMAC_BASE	(AR71XX_APB_BASE + 0x00070000)
#define QCA955X_GMAC_SIZE	0x40
#define QCA955X_GMAC_REG_SGMII_SERDES 0x0018
#define QCA955X_SGMII_SERDES_RES_CALIBRATION BIT(23)
#define QCA955X_SGMII_SERDES_RES_CALIBRATION_MASK 0xf
#define QCA955X_SGMII_SERDES_RES_CALIBRATION_SHIFT 23
#define QCA955X_SGMII_SERDES_LOCK_DETECT_STATUS BIT(15)
#define QCA955X_PLL_ETH_SGMII_SERDES_LOCK_DETECT BIT(2)
#define QCA955X_PLL_ETH_SGMII_SERDES_PLL_REFCLK BIT(1)
#define QCA955X_PLL_ETH_SGMII_SERDES_EN_PLL BIT(0)
#define QCA955X_PLL_CLK_CTRL_REG 0x08
#define QCA955X_PLL_ETH_XMII_CONTROL_REG 0x28
#define QCA955X_PLL_ETH_SGMII_CONTROL_REG 0x48
#define QCA955X_PLL_ETH_SGMII_SERDES_REG 0x4c

	void __iomem *resetbase;

static void qca955x_device_reset_clear(unsigned int mask)
{
	u32 t = ath79_reset_rr(QCA955X_RESET_REG_RESET_MODULE);
	ath79_reset_wr(QCA955X_RESET_REG_RESET_MODULE, t & ~mask);
}

static void mr18_setup_qca955x_eth_serdes_cal(unsigned int sgmii_value)
{
	void __iomem *ethbase, *pllbase;
	u32 t;

	ethbase = ioremap_nocache(QCA955X_GMAC_BASE, QCA955X_GMAC_SIZE);
	pllbase = ioremap_nocache(AR71XX_PLL_BASE, AR71XX_PLL_SIZE);

	/* To Check the locking of the SGMII PLL */
	t = __raw_readl(ethbase + QCA955X_GMAC_REG_SGMII_SERDES);
	t &= ~(QCA955X_SGMII_SERDES_RES_CALIBRATION_MASK <<
	       QCA955X_SGMII_SERDES_RES_CALIBRATION_SHIFT);
	t |= (sgmii_value & QCA955X_SGMII_SERDES_RES_CALIBRATION_MASK) <<
	     QCA955X_SGMII_SERDES_RES_CALIBRATION_SHIFT;
	__raw_writel(t, ethbase + QCA955X_GMAC_REG_SGMII_SERDES);

	__raw_writel(QCA955X_PLL_ETH_SGMII_SERDES_LOCK_DETECT |
		     QCA955X_PLL_ETH_SGMII_SERDES_PLL_REFCLK |
		     QCA955X_PLL_ETH_SGMII_SERDES_EN_PLL,
		     pllbase + QCA955X_PLL_ETH_SGMII_SERDES_REG);

	qca955x_device_reset_clear(QCA955X_RESET_SGMII_ANALOG);
	udelay(25);
	qca955x_device_reset_clear(QCA955X_RESET_SGMII);

	while (!(__raw_readl(ethbase + QCA955X_GMAC_REG_SGMII_SERDES) &
		QCA955X_SGMII_SERDES_LOCK_DETECT_STATUS));
}

#endif /* MR18_SGMII_CAL */

void __attribute__ ((noreturn)) ath_restart(void)
{
    u32 reg;
    if (soc_is_ar934x())
      reg = AR934X_RESET_REG_RESET_MODULE;
    else
      reg = QCA955X_RESET_REG_RESET_MODULE;

    for (;;) {
      ath79_reset_wr(reg, AR71XX_RESET_FULL_CHIP);
    }
}

void loader_entry(void)
{
	init_sys_type();
	init_xtimer();
	init_uart();
	printf_init(uart_putc);
	show_soc_banner();

	if (soc_is_qca955x()) {
		ANNOUNCE_LITERAL("955x SOC\n");
		qca955x_lowlevel_init();
		qca955x_init_ddr();
	} else {
		ANNOUNCE_LITERAL("9344 SOC\n");
		ar934x_lowlevel_init();
		ar934x_init_ddr();
	}

	if (test_memory((void*)KSEG1))
		goto errout;

	init_caches();
	if (test_memory((void*)KSEG0))
		goto errout;

	//initialize the malloc base.  Reserve the last meg for phram stuff
	malloc_base = (uint8_t*)(KSEG0 + ddr_find_size() - (1024*1024));
	malloc_brk = malloc_base;

	if (soc_is_qca955x()) {
	        //initialize pcie
		qca955x_init_pcie_plls();
	} else {
		//if the usb phys is still in reset, nand writes fail
		ar934x_init_usb_phy();
		//initialize pcie
		ar934x_init_pcie_plls();
	}

	ANNOUNCE_LITERAL("nand_flash_init");
	if (nand_flash_init())
		goto errout;

	ANNOUNCE_LITERAL(" ok\n");

	if (soc_is_qca955x()) {
#ifdef MR18_SGMII_CAL
		int res;

		/* even though, the PHY is connected via RGMII,
		 * the SGMII/SERDES PLLs need to be calibrated and locked.
		 * Or else, the PHY won't be working for this platfrom.
		 *
		 * Figuring this out took such a long time, that we want to
		 * point this quirk out, before someone wants to remove it.
		 *
		 * nand_init resets part of the ethernet?! so do this afterwards
		 */
		ANNOUNCE_LITERAL("MR18: Extract sgmii cal and perform it.\n");

		res = mr18_extract_sgmii_res_cal();
		if (res >= 0)
			mr18_setup_qca955x_eth_serdes_cal(res);
#else
		ANNOUNCE_LITERAL("MR18: sgmii cal not enabled...");
#endif
	}

	nand_load_fw();

errout:
	printf("error booting\n");
	ath_restart();
}
