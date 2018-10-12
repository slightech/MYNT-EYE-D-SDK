#!/usr/bin/env python
# -*- coding: utf-8 -*-


class AsciiArt(object):

  tiger = r"""
                                                     _
                                                   ,',\
                                                  ( / \\
                                                 __)\);.)
                                            _.-''  ,;',:.`-.
                                          ,'       ::(o))`. `.
                                         /          \`,',',\ `.
                                        /  ,;''._ /.--.._' /,'`.
                                     __/   /,'o\ \'.   `.`,'  `.
                                  ,';.,  `.`..--'/.`     `..:`'.
                                 `. `: \ `.`-   : |       |:'\-.`
                               _.-;`-|  `. `-.  |.:      ,'|  \ `
                            ,-'; /,' ;|   `.__.;', `.,--'\(;   `._
                          ,'; : /:  :`'.  _.-' .'    `-.:.: __    `.
                         / ; : : :  :  `/.   ,'-,'        ,` _`-._  \
                        / ; :  | :   . ,`'|,/','| \      /_,'_|>'   /
                       ; :  :  |  .  ,'      /  ; |\    ;_.-'     .'
                      :  :  :  :   ./       /  /  ; \         _.-'
                      :  :  :   `  /          /  :   `\,.__.-'
                      |  :  :     :         ,'   |
                      |  :  :     |        ;,.   ;
                      :  :   . :\ |        `  : :
                      ;  :   : | `'           | |
                    ,'|   .   .|              ; ;
                  ,'; :   :    |             : :
                ,; ; ';  . .SSt;/            ; |


              Wow... That's a lot of linter errors...

  """

  cthulhu = r"""


                                    _.---._
                                .-'         '-.
                             .'                 '.
                            '       '.   .'       '
                           / /        \ /        \ \
                          '  |         :         |  '
                         /   |         .         |   \
                         |   \         |         /   |
                         '. . \        |        / . .'
                          |   .\      .'.      /.   |
                          \  .  `-           -'  .  /
                           '.      .. ... ..      .'
                            |  `` ` .     . ` ``  |
                            | .-_-.  '. .'  .-_-. |
                           .'( (O) )|  :  |( (O) )'.
                            \.'---'/   :   \'---'./
                              \_ .'  . ' .  '. _/
                             .' /             \ '.
                             './ / /  / \ \  \ \.'
                              : | | /|  : |  | :
                              | : | \\  | '  : |
                              | /\ \/ \ | : /\ :
                              ' :/\ \ : ' ||  \ \
                              / | /\ \| : ' \  \ \
                             / / /  \/ /| :  |  \ \
                            / / :   / /\ \ \ /   \ \
                           ' /\ \  | /\ :.\ \    / |
                           \ \ \ \ \/ / || \ \   \/
                            \/  \|    \/ \/ |/ LGB


            I once knew somebody who tried to commit lousy code like this...

  """

  story = r"""
              ______        _         ______        _
              \___  \____  | |        \__   \____  | |
               | | _/\__ \ | |__       | | _/\__ \ | |__
               | |  \ / _ \| _  |      | |  \ / _ \| _  |
               |____/(____/|_||_| /\   |____/(____/|_||_| /\
          ______ _              _ )/    _____ _           )/
          \___  \ | ____   ___ | | __  /  __/| |___  ___  ___  ____
           | | _/ | \__ \ / __\| |/ /  \__ \ |  _  \/ _ \/ _ \|  _ \
           | |  \ |__/ _ V  \__|   <   / /  \| | | |\  _/\  _/| |_> )
           |___ /___(____/\___/|_|__\ /____/ |_| |_| \___\\___\  __/
    ##########################################################| |##########
    #                                ___       _              `-'       __#
    # Bah, bah, black sheep,        (_  `)    ( `)_        _//(   ,    (_ #
    #   Have you any wool?           (____)  (_____)      oo\\\\,' `._   (#
    #    ,`.                                        __    <_.\\\--._,``.  #
    #  ,`-._`.           ,@;@,  Yes sir, yes sir,  /_ \    \__/"  ' `   `.#
    #,'     ,@;@;@;@;@;@/ )@;@;  Three bags full. (( ||  .`//_`.       `  #
    #   , ,;@;@;@;@;@;@|_/@' e\       _            `'||,' ||/ )| |   \    #
    #  , (|@;@:@\@;@;@;@:@(    \ _.--' ' .         ,-||___|/ / |          #
    #      '@;@;@|@;@;@;@;'`"--'_.     .   `'- - -' (______.'H |'  '   `  #
    #   ,  `'@;@;/;@;/;@;'                 '      ,  ||   #==#==          #
    #       ' ) //   | || `._   \|/   \//            || , || V | One   `  #
    #    '    \ \\ ` | ||  `  '-           \|/   .   ||   ||   |' for     #
    #  '  \\/  \ \\  ) \\ ` \\/ `  \\|.            __||   ||   |   the  ` #
    #  \|\\|\//\`"`  `"``/||/| \         \//    ,-'  ||`-.||___| master,  #
    #                             \\/                ||   _||_|-._.`-.__.`#
    #      )).__   One      \\|/         \|//   _____()__<<____)__________#
    #  ^^ ((  __)   for    |_._._._._._._._._._|                          #
    #     (_(//o(    the              And one                         ^^  #
    # ^^   (_('_/     Dame,   ((())   for the       ,:;:,   ^^            #
    #     __/_(__ ^^         ( . . )  little boy   ;:;.( \:;:@:;:;:;:,    #
    #    /\\___//\          (|  v  |)              /a `:\_|:;.;:;@;:.@;,  #
    #~~~/_/\  * /_\~~~~~~~~~~| '-' |~~Who lives---/    )::.;@;:;:/@::;:|)-#
    #~ / /\|*  *|\ \ ~~  ~  ~ \___/~  down the \/ `--"'`;@;:;:;:|.;:;@;`  #
    #  \ \ \_*_/ ~\ \ ~ ~  ~ __| |___  lane.;;           `':;\;:;\;@;:`   #
    # ~ \_)/* *\~  \_)~  ~  /"":-:"''\  ~  .;;  Bah, Bah,  | | |   \\ (   #
    #  ~  /* * *\ ~/__\ ~  | | : : |. |~ ~ ;;    black     || |   // /    #
    #~ ~ / \*_*_|\|`--`| ~ | |  :  || | ~ ,;;     sheep,   // (  // / \\| #
    #mmm/_________|    |mmm|_|     ||_|mm,,;  \\/          ''"'  '"'      #
    #      | | |  '----'   /_|""''"||_\,;,;   _......_       Have you     #
    # \||/ \ ) )  \|/        |  _  |,', ,'\/ ';-.---';'        any wool?  #
    #      / | |      \|//   | |.| |, ,'       }====={ \|/   _.---.._     #
    #------\_|_|-----------_.|_|,|_|,'        .'  _   '.    ';-..--;'     #
    #      Y\_)_)      _.-`,/__|,|__\   \|/  /:: / |    \    `}===={  \// #
    #--\|/---------_.-`, ,  , ,_.'          |::   ||     | .:`  __  '.    #
    #          _.-`  ,   , .-'`   \|//      \::. _||_   ,/;:__ (-,\   \   #
    #\|\\/ .-'`,  ,   ,  ,'             \|/  '::_`""` _-;'--.-'; //    |  #
    #    .'   ,  ,  , , /   \\|/                 `````  }====={ /(_    /  #
    #\|,' , ,  ,  ,    /                    Yes sir,  .'  .-.  '.""`_.'   #
    # /  ,   ,   , ,  /         \\|//       yes sir  /:: (( ))   \``      #
    #/ ,  ,   ,  ,   /  \|/              \|/        |::    <<     | \|/   #
    #'  '  '   ,  , ,              \\/       Three  \::. ((_))    /       #
    # '   '  '  '   |         \//              bags  '::_ `"`  _.'        #
    #   '   '     ' |  \\|/           \|/        full.   ``````     \\\/  #
    ##########dwb#########jgs########Krogg#######JRO#########mga###########

  """

  commit_success = r"""

            d888888b                         d888888b
         d888    8888b                    d888888   888b
       d88    88  898888b               d8888  888     88b
      d8P        88888888b             d88888888888     b8b
      88        8888888888             88888888888       88
      88       88888888888             8888888888        88
      98b     88888888888P             988888888        d8P
       988     888  8888P      _=_      9888898  88    88P
         9888   888888P      q(-_-)p       98888    888P
            9888888P         '_) (_`         9888888P
               88            /__/  \            88
               88          _(<_   / )_          88
              d88b        (__\_\_|_/__)        d88b

        Your happiness shall not depend on what you have or what your are,
         it shall solely depend on the beauty of your code.
         
  """

  grumpy_cat = r"""

                                                   : sdMMm+
                                                /mMMMMMMM+
      /Mms-                                   -mMMMMMMdsMm
      .NMMMd:                               `yMMMMMmdM+ sM`
       oMNmMMmo-        `.-::::::-.`       +NMMMMmo:mM--hM`
        sMNmmMMMh//+++++/:-......-:/osyo/sNMMMmyo+-..yNMMN
         hNmNmys+/                     :ooyNMMd:    `-MMMy
         `mMms         `                   .ohNo:` .+NMMm`
          :MMh  .../:+yy   -  :/              ./hMhhmNMM:
          -NN`-yMMMMMMh-h :M:+MMh/.             .hMMMNdd
         -NMMNMMMMMMMM:dd oMMMMMMMN.             oMMMMNs
        .NMMMMNNMMMMMN``  /MMMMMMMMmys+/+/.      :MMMMNo
        sMMMM++N.-dMMN`   -MMMMMm/dm-:yMMMNs-     .oNMM:
        NMMMMo-s.:dMN:     /MMMM: +h``hMMMMMMy      `hM+
       `MMMMMMMMMMMms+/.   -NMMMNhsoyNMMMMMMy`       oMM`
       -dmMMMMMMdo.dMMMMh`  .hMMMMMMMMMMMMMM+        -MM`
       -MNdddh/`    -yMy.     .+hNMMMMMMMMMMN         hM-
       .dmmmd`   -shyoooyys/     `:hMMMMMMMN/         :Ms
       `MMNdy   +d-        :y.     `ydddddo`           dN`
        ydmNM+  m`          `y:     /dddds             :M/
       `MMMMMMmh+`            /:   +dddNMm-             dd
        MMMMMMMMh:             -smNMMMMmdds             -M.
       `MMMMMMMMMMms+/--..--      ./oyyys+`              my
       -MMMMMMMMMMMMNhoo+/:.                             sM-
       /MMMMMMMMMMMMMMNy+:---`                           `mm`
       oMMMMMMMMMMMNMMMNMMMMm:                            -Nh`
       sMMMMMMMMMMNds.` `...                               :My
       mMMMMMMMMMMy/`                                       /Mo
      :MMMMMMMMMMMMMNy`                                      +M-
     .mMMMMMMMMMNdhy--                                        dh
    -NMMMMMMMMMMMMMMMMNyo.                                    :M

      oMMN/     hMMs  -yNMMMMMMMMh/  .MMMMMMMMMNs`dMMMMMMMMMM.
      oMMMMy`   hMMs`yMMMh+:-:/yNMMd..MMMs:::+mMMdmMMh:::::::`
      oMMMMMm-  hMMsyMMm-       `hMMm-MMM/    -MMMMMMy
      oMMmyMMM+ hMMdMMM:         `NMMoMMM/```-yMMNmMMNmmmmmmm`
      oMMd /NMMhdMMhMMM/         `NMMoMMMMMMMMMMd-dMMNmmmmmmd`
      oMMd  .dMMMMMsyMMN:       .hMMm.MMMhssss+-  dMMy
      oMMd    sMMMMs sMMMdo/::+hMMMd..MMM/        dMMd///////`
      oMMd     :NMMs  -yNMMMMMMMMh:  .MMM/        dMMMMMMMMMM-
      .//:      `//-     .:+++/-      :::`        ://////////`

      One shan't commit directly to master!

  """

  homer_woohoo = r"""

                                    ,#M]RRD`"RW
                                   ,R,qB*""``"*Rw,
                                  ,#M 6          `%w
                                 ]M                "N
                                ]M                  ]H
                                8                    Rw
                                        aR*"*%w wR*"RW#H
            ,,                  ]     ;R       TL     1m                 ,,w,
       ,#K#C  `%p                N ,  ] ]R      R    BNB                ]R  #R^*N
     w]C   ]www,#               qR,R1w"N       /RMM*%mR               ,#M``RW  ,/R,
        TW#^     8              R]M  "  *Wwww#M`   ,#M               ]M y     M    1
     @    D  ,q  ]L             ,dNw    ,wmRM**"``"```"Rw,,          ]H  "Rm#R  yR ]
    9QRK4M`""`    Rw            R5R9  ,R"               T  B       ,#"    R   ``` zR
     "N             RW           *R#  @                #RHR      aR^           ,#M
       "RW            "Kw          R  0               w,w@M   ,#M            zR`
          Tw             "N,       #  'N            ,R"    ,gM`            ,R`
           `Rw              "Nw ,  #    %W,      ,@R` ,w,qR^             ,#"
             `0,               "R"NDwwww=m#RRRRR" ],,#M  `*N,          ,#M
               "#,             ]R  "N       m"K,  ]UR N     `Rw       dR
                 "N           ]R     0     A   `0w]RN 1@       Rw   /R
                   "N       ,R`       9m==4*     `   R#R`       "NzR
                     "N   ,R^          `              9          ]R
                       "RR`                            N       ,R^
                        `RW,                           "N  ,qR"
                            "RWmw==                     `*N
                             xR                            "N
                           zR                                1w
                         ,R`                                  1N
                        /R                                     1L
                       /R

                    Woohoo, no formatter or linter for merge commits!

  """
