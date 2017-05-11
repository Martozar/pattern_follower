# Následování člověka mobilním robotem
# Mykhaylo Zelenskyy
# 2016/2017

### Před instalací

Pro použití tohoto programu je třeba mít nainstalovanou knihovnu OpenCV v. 3.1.
Stáhnout OpenCV lze [zde](http://opencv.org/releases.html).

Zajistěte si, že máte poslední kompilátor GCC 4.9.0 (a novější) nebo Clang 3.5.0 (a novější), protože se používá standart c++14.

Pokud chcete používat dálkoměr Hokuyo, stáhněte si a nainstalujte [HokuyoAIST](https://github.com/gbiggs/hokuyoaist) a [Flexiport](https://github.com/gbiggs/flexiport).

### Instalace

Vytvořte novou složku.

```
mkdir build && cd build
```

Dále pokud používáte laserový dálkoměr použijte

```
cmake ..
```

V případě, pokud nechcete dálkoměr používat, pak

```
cmake -DWITH_LASER=OFF ..
```

Potom snačí již nainstalovat knihovnu

```
make install
```

### Použití

Spustit program lze příkazem

- `$ pattern_follower -c <path_to_config>`

Pokud nebude přidána cesta ke konfiguračnímu souboru, bude použit soubor ze složky, ve které byla vytvořena složka build.
