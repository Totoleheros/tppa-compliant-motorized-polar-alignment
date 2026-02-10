# Polar Align System – Hardware Setup

## ⚠️ Disclaimer

> I'm just an enthusiast sharing this open hardware project, **with no guarantee of success**.  
> I’ll do my best to support others trying this build, but my **time is limited**, and my **skills are not professional-grade**.  
> This is an **early prototype** and proof of concept — not fully validated yet. I hope to share updated iterations in the future.

## 🧩 3D Model & Files

- The full 3D design is available here:  
  👉 [Shapr3D Project Viewer](https://collaborate.shapr3d.com/v/ZjITyAcu-yg3d5nvveWZC)

- All 3D parts (STEP format) are included in the downloadable archive:  
  📦 `PolarAlignSTEP.zip`

## 🛒 Hardware Components

### 1. Tripod Extension
- Example: [AliExpress – 43€](https://fr.aliexpress.com/item/1005008669077575.html)
- ![Tripod Extension](IMAGES/tripod-extension.jpg)

### 2. Harmonic Drive (AZM Motor)
- Model: **MINIF11-100**
- Example: [AliExpress – 58€](https://fr.aliexpress.com/item/1005007712296652.html)
- ![Harmonic Drive](IMAGES/harmonic-drive.jpg)

### 3. Stepper Motors (x2)
- Model: **17HS19-2004S1**
- Example: [Amazon – 36€ for 3 units]
- ![Stepper Motor](IMAGES/stepper.jpg)

### 4. Cross Slide Mini Lathe Compound Table
- Example: [AliExpress – ~85€](https://fr.aliexpress.com/item/1005004961267751.html)
- ⚠️ You may need a shorter belt: used belt = **XL118**
- ![Lathe Slide](IMAGES/lathe-slide.jpg)

### 5. Main Controller Board
- Board: **FYSETC E4 V1.0** (⚠️ **Pin mapping differs** on V2.0!)
- Features: WiFi + Bluetooth, 4x TMC2209, 240MHz, 16M Flash
- Example: [AliExpress – ~30€](https://fr.aliexpress.com/item/1005001704413148.html)
- ![FYSETC E4 Board](IMAGES/fysetc-e4.jpg)

### 6. Orientation Ring (iglidur® PRT-02)
- **Reference**: [igus PRT-02 LC J4](https://www.igus.fr/product/iglidur_PRT_02_LC_J4) (~€63)  
  ![Orientation Ring](IMAGES/orientation_ring.jpg)
---

### 💰 Estimated Total: ~**315€**

## 🖨️ 3D Printing Notes

- All 3D parts were printed in **100% infill PLA** during prototyping.
- It is **highly recommended** to **CNC machine** the 2 **blue-colored parts** from the 3D model:
  - These parts bear the full mechanical load from the telescope and mount.
  - Estimated machining cost: ~**80–90€**
- The **ALT motor bracket** can optionally be metal.
- ![Load-bearing Parts](IMAGES/load-bearing.jpg)

### 🧮 Final Cost Estimate

- ~320€ hardware base
- + CNC machining (optional): ~90€
- 🟰 **~400-450€ total project budget**

---

## 🔌 Wiring & Resources

- **OnStep E4 wiring inspiration**:  
  [OnStep Wiki – PDN & jumpers](https://onstep.groups.io/g/main/wiki/32747)
  > *Remove all factory jumpers. Connect only Z-Min to PDN for UART setup.*

- **FYSETC Wiki (E4 Board)**:  
  [https://wiki.fysetc.com/docs/E4](https://wiki.fysetc.com/docs/E4)

---

## 📸 Assembly Photos

You can find detailed images in the `/IMAGES/ASSEMBLY` folder of this repository.

---

Feel free to open issues if you need clarification or want to improve this documentation.
