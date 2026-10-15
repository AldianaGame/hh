# Astrea Razor — Build APK melalui GitHub Actions

Proyek ini mempertahankan sumber React/Vite dari Google AI Studio dan membungkus hasil build web ke aplikasi Android WebView.

## Cara pakai dari HP Android
1. Ekstrak ZIP ini.
2. Unggah seluruh isi folder proyek ke root repository GitHub (jangan unggah folder pembungkus `AstreaRazor-ready` sebagai satu folder di dalam repository).
3. Pastikan `package.json`, `src/`, `index.html`, `settings.gradle`, `build.gradle`, `app/`, dan `.github/workflows/build.yml` ada di root repository.
4. Commit ke branch `main` atau `master`.
5. Buka tab Actions → `Build Astrea Razor APK` → tunggu sampai hijau.
6. Buka run yang sukses → bagian Artifacts → unduh `Astrea-Razor-debug-apk.zip`.
7. Ekstrak ZIP dan pasang `app-debug.apk`. Jika Android meminta izin, izinkan pemasangan dari sumber tersebut.

## Catatan
- Workflow menjalankan `npm install`, `npm run build`, menyalin `dist/` ke Android assets, lalu menjalankan `gradle assembleDebug`.
- APK menampilkan aplikasi web yang dibangun dari sumber proyek ini.
- Ini membungkus UI web; koneksi nyata ke Wemos D1 Mini hanya berfungsi jika kode web/firmware sudah mengimplementasikan komunikasi Wi-Fi yang cocok.
- Jangan masukkan API key atau rahasia ke repository publik.
